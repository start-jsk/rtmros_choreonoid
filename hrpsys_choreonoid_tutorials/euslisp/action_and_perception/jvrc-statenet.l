(require :jvrc-standup "jvrc-standup.l")

;; ------------- copy from euslib -------------
;;
;; StateNet (using irtgraph.l)
;;

;;
;; State
;;
(defclass robot-state
  :super propertied-object
  :slots (
          coords          ;;;ロボットの姿勢を表すcoordinatesのインスタンス
          pose            ;;;ロボットの姿勢(駆動する関節角ベクトル)
          torque-vector   ;;;ロボットの関節トルク
          extra-vector    ;;;ロボットのその他のセンサデータ(力センサなど)
          weight-vector
          )
  )

(defmethod robot-state
  ;;
  (:init
   (p c &key ((:torque tq)) ((:extra r)) ((:weight w)))
   (setq pose p
         coords c
         torque-vector tq
         extra-vector r
         weight-vector w)
   self)
  ;;
  ;; accessor
  ;;
  (:posture (&optional p)(if p (setq pose p) pose))
  (:extra-vector (&optional p)(if p (setq extra-vector p) extra-vector))
  (:torque-vector (&optional tq)(if tq (setq torque-vector tq) torque-vector))
  (:coords (&optional c) (if c (setq coords c) coords))
  (:weight (&optional w)(if w (setq weight-vector w) w))
  ;;
  (:raw-vector () (concatenate float-vector pose (send self :pitch-roll) torque-vector extra-vector))
  (:vector (&optional (overwrite-weight :none)) ;; weighted vector
   (let ((vec (send self :raw-vector))
         (w weight-vector))
     (unless (eq overwrite-weight :none)
       (setq w overwrite-weight))
     (when w
       (setq vec
             (map float-vector #'(lambda (a b) (* a b)) vec w)))
     vec))
  (:norm () (norm (send self :vector)))
  (:pitch-roll ()
   (let ((rpyangles (car (rpy-angle (send coords :rot)))))
     (float-vector (- (elt rpyangles 1)) (elt rpyangles 2))
     ))
  #|
  (:pitch-roll ()
   (let* ((z #f(0 0 1))
          (y #f(0 1 0))
          (lz (send coords :rotate-vector z))
          (lx (send coords :rotate-vector #f(1 0 0)))
          (ly (send coords :rotate-vector y))
          (nz (v* z lz))
          (ny (v* z ly))
          )
     (normalize-vector nz nz)
     (normalize-vector ny ny)
     ;; (print (list lz ly nz ny))
     (if (< (v. lx z) 0) (setq nz (v- nz)))
     (float-vector (vector-angle z lz nz)
                   ;;(vector-angle z ly ny)
                   (v. z ly)
                   )
     ))
  |#
  ;;
  (:posture-distance (state)
   (norm (v- (send state :posture) (send self :posture))))
  (:rotation-distance (state)
   (norm (v- (send state :pitch-roll) (send self :pitch-roll))))
  (:extra-distance (state)
   (norm (v- (send state :extra-vector) (send self :extra-vector))))
  ;; Mahalanobis' Distance
  ;; Chebyshev distance
  ;; euclid distance
  ;; manhattan
  (:distance
   (other-state &key (overwrite-weight :none))
   (let ((w weight-vector))
     (unless (eq overwrite-weight :none)
       (setq w overwrite-weight))
     (norm (v- (send self :vector w)
               (send other-state :vector w)))
     ))
  (:distance-Mahalanobis
   (other-state &key (overwrite-weight :none) (inv-covariance-matrix))
   (let ((w weight-vector))
     (unless (eq overwrite-weight :none)
       (setq w overwrite-weight))

     (let* ((vv (v- (send self :vector w)
                    (send other-state :vector w))))
       (unless inv-covariance-matrix
         (setq inv-covariance-matrix
               (unit-matrix (length vv))))
       (pprint (transform inv-covariance-matrix vv))
       (pprint (v. vv (transform inv-covariance-matrix vv)))
       (sqrt (v. vv (transform inv-covariance-matrix vv))))
     ))
  (:midstate
   (astate &optional (ratio 0.5))
   "if ratio := 0, returns self
    if ratio := 1, returns astate"
   (let ((apose (send astate :posture))
         (atorque (send astate :torque-vector))
         (aextra (send astate :extra-vector))
         (aweight (send astate :weight))
         (acds (send astate :coords)))
     (instance robot-state :init
               (if (and pose apose) (midpoint ratio pose apose))
               (if (and coords acds) (midcoords ratio coords acds))
               :torque (if (and torque-vector atorque) (midpoint ratio torque-vector atorque))
               :extra (if (and extra-vector aextra) (midpoint ratio extra-vector aextra))
               :weight (if (and weight-vector aweight) (midpoint ratio weight-vector aweight)))
     ))
  (:prin1 (&optional (strm t) &rest msgs)
   (send-super :prin1 strm
               (format nil "coords ~A pose ~A" coords pose)))
  )

;;
;; Node
;;
(defclass state-node
  :super node
  :slots (
          robot
          state ;; instance of robot state
          threshold
          ;; weight
          )
  )

(defmethod state-node
  (:init
   (r s &key (name) ((:threshold thr)))
   (send-super :init name)
   (send self :robot r)
   (send self :state s)
   (send self :threshold thr)
   self)
  ;;
  ;; accessor
  ;;
  (:robot (&optional r) (if r (setq robot r)) robot)
  (:state (&optional s) (if s (setq state s)) state)
  (:threshold (&optional thr) (if thr (setq threshold thr)) threshold)
  ;;
  (:cost () 0)
  (:prin1 (&optional (strm t) &rest msgs)
   (send-super :prin1 strm
               (format nil "robot ~A state ~A" robot state)))
  )

(defun state-distance-Euclidean (state1 state2 &key (print) (overwrite-weight :none)
                                        &allow-other-keys)
  (derivedp state1 state-node)
  (let* ((s1 (if (derivedp state1 state-node) (send state1 :state) state1))
         (s2 (if (derivedp state2 state-node) (send state2 :state) state2))
         (dist (send s1 :distance s2 :overwrite-weight overwrite-weight)))
    (when print
      (print (list dist state1 state2)))
    dist
    ))
(defun state-distance-Mahalanobis (state1 state2 &key (print) (overwrite-weight :none)
                                          (inv-covariance-matrix) &allow-other-keys)
  (let* ((s1 (if (derivedp state1 state-node) (send state1 :state) state1))
         (s2 (if (derivedp state2 state-node) (send state2 :state) state2))
         (dist (send s1 :distance-Mahalanobis s2
                     :overwrite-weight overwrite-weight
                     :inv-covariance-matrix inv-covariance-matrix
                     )))
    (when print
      (print (list dist state1 state2)))
    dist
    ))
;;
;; Arc
;;
(defclass state-arc
  :super costed-arc
  :slots ()
  )
(defmethod state-arc
  (:init (from_node to_node &key name ((:cost c) 0))
   (send-super :init from_node to_node c)
   (send self :name name)
   self)
  (:cost (&optional c) (if c (setq cost c)) cost)
  (:source () from)
  (:destination () to)
  )

;; definitions for net
(if (not (constantp '*state-max-error*))
    (defconstant *state-max-error* 100))
(defconstant *exit-error-count* 3)
(defvar *verbose-statenet* nil)

;;
;; needed functions for state net
;; sense-current-state () returns [robotstate]
;; transit-to-state (s-node)
;; do-action (state-arc) returns :fail or :success
;;
(defun sense-current-state (&rest args)
  ;; make robot-state from sensor
  (warn ";; you should over write function 'sense-current-state' for your robots!!~%")
  (instance robot-state :init
            (concatenate float-vector (random-vector 10.0) (random-vector 10.0) (random-vector 10.0))
            (make-coords :pos (random-vector 1000.0) :rpy (random-vector pi)))
  )
(defun transit-to-state (s-node)
  (warn ";; you should over write function 'transit-to-state' for your robots!!~%")
  )
(defun do-action (arc)
  (let* ((func (read-from-string (format nil "~A-state-action" (send arc :name)))))
    (if (fboundp func)
        (funcall func)
      (progn ;; function is not existing
        (warn ";; function ~A is not existing~%" func)
        :fail))))

;;
;; Net
;;
(defclass state-net
  :super graph
  :slots (
          robot           ;;;ロボット
          current-state   ;;;現在の状態
          error-count
          sense-current-state-func
          state-distance-func
          state-distance-args
          )
  )

(defmethod state-net
  ;;
  ;; init
  ;;
  (:init (r &key (current-state-function))
   (setq robot r
         error-count 0
         sense-current-state-func current-state-function)
   (unless sense-current-state-func
     (setq sense-current-state-func 'sense-current-state))
   (setq state-distance-func 'state-distance-Euclidean)
   self)
  ;;
  ;; accessor
  ;;
  (:robot (&optional r)
   (when r
     (setq robot r)
     (send self :nodes :robot robot)
     (send self :arcs :robot robot))
   robot)
  (:current-state (&optional s)
   (when s
     (let ((ret (send self :find-node s)))
       (if ret (setq current-state ret))))
   (send self :display-current-state)
   current-state)
  (:state-distance-func
   (&optional func)
   (when func
     (setq state-distance-func func))
   state-distance-func)
  (:state-distance-args
   (&optional args)
   (when args
     (setq state-distance-args args))
   state-distance-args)
  ;;
  (:set-covariance-matrix () ;; for Mahalanobis' Distance
   (let* ((mat (apply #'matrix
                      (mapcar #'(lambda (nd) (send (send nd :state) :vector)) (send self :nodes))))
          (dims (array-dimensions mat))
          ;;(row (car dims))
          (col (cadr dims))
          ret)
     (dotimes (i col)
       (push (matrix-column mat i) ret))
     (setq ret (covariance-matrix (nreverse ret)))
     (setq state-distance-args
           (list :inv-covariance-matrix
                 (inverse-matrix ret)))
     ))
  ;;
  (:add-arc
   (from to &key (name) ((:cost c) 0))
   (send self :add-arc-from-to from to c :name name))
  (:add-arc-by-name (from-nm to-nm &key (name) ((:cost c) 0))
    (let ((from (send self :find-node from-nm))
          (to (send self :find-node to-nm)))
      (if (and from to)
          (send self :add-arc-from-to from to c :name name)
        (warn ";; can not find node ~A ~A~%"
              (list from-nm from) (list to-nm to)))
      ))
  (:add-arc-from-to (from to cost &key (name))
   (let ((func (read-from-string (format nil "~A-state-action" name))))
     (unless (fboundp func)
       (warn ";; function (~A) is not defined~%" func)
       (return-from :add-arc-from-to)))
   (instance state-arc :init from to :cost cost :name name))
  (:find-node-by-name (node-name)
   (find-if #'(lambda (nd) (string-equal (string node-name)
                                         (string (send nd :name)))) nodes))
  (:find-node (nn)
   (cond
    ((derivedp nn state-node)
     (if (memq nn nodes) (return-from :find-node nn)
       (progn (warn "~%") (return-from :find-node)))
     )
    ((or (symbolp nn) (stringp nn))
     (let ((ret (send self :find-node-by-name nn)))
       (if ret (return-from :find-node ret)
         (progn (warn "~%") (return-from :find-node)))
       )))
   (warn "~%")
   nil)
  ;;
  ;; display
  ;;
  (:display-action-queue (queue)
   (let ((str "ACTION-QUEUE:"))
     (dolist (q queue)
       (setq str
             (format nil
                     "~A~A:=(~A)=>"
                     str
                     (send (send q :source) :name)
                     (send q :name))))
     (if queue
         (setq str
               (concatenate string
                            str
                            (string (send (send (car (last queue))
                                                :destination) :name)))))
     (format *error-output* "~A~%" str)
     ))
  (:display-current-state ()
   (format *error-output* ";; Current State is ~A~%" (send current-state :name))
   )
  ;;
  ;; search
  ;;
  (:search-paths
   (from to)
   (send self :start-state from)
   (send self :goal-state to)
   (let ((solve-wf (instance breadth-first-graph-search-solver :init))
         (solve-df (instance depth-first-graph-search-solver :init))
         (solve-bf (instance best-first-graph-search-solver :init self))
         result ret)
     (push (send solve-wf :solve self) result)
     (push (send solve-df :solve self) result)
     (push (send solve-bf :solve self) result)
     (setq ret
           (mapcar #'(lambda (res)
                       (mapcar #'(lambda (path) (send path :action)) (cdr res)))
                   result))
     ;; remove nil
     (remove nil ret)
     ))
  (:find-lowest-cost-path
   ;;複数のパスから最もコストの小さいパスを返す．
   (paths)
   (find-extream paths #'(lambda (x) (send self :statenet-path-cost x)) #'<=)
   )
  (:search-lowest-cost-path
   ;;stateクラスのインスタンスを与えて最小コストのパスを求める．
   (from to)
   ;;(format t "search-lowest-cost-path:from:~A,to:~A~%" from to)
   (send self :find-lowest-cost-path
         (send self :search-paths from to)))
  (:search-lowest-cost-path-by-name
   ;;stateクラスのインスタンスのnameを与えて最小コストのパスを求める．
   (from to)
   (send self :search-lowest-cost-path
         (send self :find-node-by-name from)
         (send self :find-node-by-name to)))
  (:statenet-path-cost
   ;;パスのコストを求める．
   (path)
   (apply #'+ (mapcar #'(lambda (x) (send x :cost)) path)))
  ;;
  ;;
  ;;
  (:do-action (ac &key (print *verbose-statenet*))
   (send self :go-state (send ac :source) :print print)
   (send self :do-action-sequence (list ac) :print print)
   )
  (:do-action-by-name (acname)
   (let ((ac (send self :find-arc acname)))
     (if ac (send self :do-action ac)
       (format *error-output* "No such action(~A).~%" acname))
     ))
  ;;
  (:go-state
   (to &key (toplevel t) (print *verbose-statenet*))
   (if toplevel (setq error-count 0))
   (unless current-state
     (let* ((rs (funcall sense-current-state-func :mode :recover))
            (st (send self :find-nearest-state rs)))
       (if (send self :on-state st)
           (send self :current-state st)
         (progn
           (warn ";; can not determine current state~%")
           (return-from :go-state)
           ))
       ))
   (let* ((dstate (send self :find-node to))
          (path (send self :search-lowest-cost-path current-state dstate)))
     (if path
         (format *error-output* ";; go-state to ~A count: ~d~%"
                 (send dstate :name) error-count)
       (format *error-output* ";; path not found to ~A~%"
               (send dstate :name))
       )
     (send self :do-action-sequence path
           :toplevel toplevel :print print)
     ))
  ;;
  (:do-loop (state1 state2)
   (while t
     (if (equal :stop (send self :go-state state1))
         (return-from :do-loop nil))
     (if (equal :stop (send self :go-state state2))
         (return-from :do-loop nil))
     ))
  (:do-loop-by-name (state1 state2)
   (send self :do-loop
         (send self :find-node-by-name state1)
         (send self :find-node-by-name state2)))
  ;;
  (:on-state (astate &optional (overwrite-threshold))
   (let ((rs (funcall sense-current-state-func :mode :check))
         (thr (send astate :threshold))
         dist)
     (if overwrite-threshold
         (setq thr overwrite-threshold))
     (setq dist (apply state-distance-func astate rs state-distance-args))
     (warn ";; on-state (~A) dist: ~A, threshould: ~A~%" (send astate :name)
           dist thr)
     (unless thr
       (warn ";; no threshold~%")
       (return-from :on-state t))
     (< dist thr)
     ))
  (:check-current-state ()
   (send self :on-state current-state))
  ;;
  (:do-action-sequence
   (acs &key (toplevel t) (print))
   (let (ac to)
     (if acs (setq to (send (car (last acs)) :destination)))
     (while acs
       (send self :display-action-queue acs)
       (setq ac (pop acs))
       (case (do-action ac)
        (:fail
         (format *error-output* ";; Action failed! (~A) dist: ~6,4F~%"
                 (send ac :name)
                 (setq dist (send self :find-nearest-state-dist
                                  (funcall sense-current-state-func) :print print)))
         (if toplevel
             (let ((r
                    (send self :recover-to-state to :print print)))
               (if r (setq r current-state))
               (return-from :do-action-sequence r))
           (return-from :do-action-sequence)))
        (t
         (send self :current-state (send ac :destination)))
        )
       ;; check current-state is fine
       (unless (send self :check-current-state)
         (warn ";; Current is not on state (~A)~%" (send current-state :name))
         ;; act as the same action as :fail returned
         (if toplevel
             (let ((r
                    (send self :recover-to-state to :print print)))
               (if r (setq r current-state))
               (return-from :do-action-sequence r))
           (return-from :do-action-sequence)))
       ) ;; while
     current-state))
  (:recover-to-state (s &key (print))
   (format *error-output* ";;   Recover to state (final destination ~A)~%" (send s :name))
   (while (not (eq s (send self :current-state (send self :recover-to-net :print print))))
     (if (> error-count *exit-error-count*) (return-from :recover-to-state))
     (incf error-count)
     (when (eq s (send self :go-state s :toplevel nil :print print))
       (return-from :recover-to-state t))
     )
   (eq s current-state))
  (:recover-to-net (&key (print))
   (let* ((cs (funcall sense-current-state-func :mode :recover))
          (ns (send self :find-nearest-state cs :print print)))
     ;;(format *error-output* ";; Trying to recover to NET (~A)~%" (send ns :name))
     (while (not (send self :try-transit-to ns :print print))
       (setq cs (funcall sense-current-state-func :mode :recover)
             ns (send self :find-nearest-state cs :print print))
       ;;(if (eps= 0.0 (norm (v- (send cs :posture) (send ns :posture))))
       ;;(error ";; unable to recover to StateNet~%"))
       ;;(state-distance ns cs :print print)
       )
     ns))
  (:try-transit-to (s &key (print))
   (let (ret)
     (format *error-output* ";;     Trying transit to (~A)~%" (send s :name))
     (transit-to-state s) ;; state に移行
     (let ((r (send self :on-state s))) ;; transitに成功したか確認
       (if r
           (warn ";;     -> successed to transit to (~A)~%" (send s :name))
         (warn ";;     -> faild to transit to (~A)~%" (send s :name)))
       r)
     ))
  (:find-nearest-state
   (rs &key (print))
   (let ((ns (remove rs (send self :nodes))) node)
     (setq node (find-extream ns #'(lambda (x)
                                     (apply state-distance-func x rs :print print state-distance-args)
                                     )
                              #'<=))
     (when print
       (format *error-output* "find node ~A~%" (send node :name))
       (format *error-output* "current: " (send node :name))
       (dump-structure *error-output* rs)
       (format *error-output* "~%" (send node :name))
       )
     node
     ))
  (:find-nearest-state-dist
   (rs &key (print))
   (let ((ns (remove rs (send self :nodes))) node)
     (apply 'min (mapcar #'(lambda (x)
                             (apply state-distance-func x rs :print print state-distance-args)) ns))
     ))
#|
  (:match-state (s &key (print))
   (let ((ns (send self :nodes))
         node)
     (setq node
           (find-extream ns
                         #'(lambda (x) (send s :error-distance
                                             (send x :state) :print print)) #'<=))
     (if print (format *error-output* "find node ~A ~%" (send node :name)))
     (send s :error-distance (send node :state))
     ))
|#
  ;;
  (:print-definition (&optional (strm t))
   (format strm "(let* ((robot (~A))~%" (send robot :name))
   (format strm "       (net (instance state-net :init robot))~%")
   (format strm "       act)~%")
   (dolist (a arcs)
     (format strm "(setq act ")
     (send a :print-definition strm)
     (format strm ")~%")
     (format strm "(send net :add-action act)~%")
     )
   (format strm "net)~%")
   )
  )

;; end ------------- copy from euslib -------------

;; overwrite function
(defun sense-current-state (&key (mode) (average) (hz 10.0) (weight))
  (send *ri* :state)
  (let ((potentio (send *ri* :potentio-vector))
        (cds (send (send *ri* :robot) :copy-worldcoords))
        ;; (torque (send *ri* :torque-vector))
        #|(extra (apply #'concatenate float-vector
                      (append (send *ri* :off-force-vector) (send *ri* :off-moment-vector)
                              (list (send *ri* :accel-vector)))
                      ))|#
        )
    (when average
      (let ((pt (list potentio))
            ;;(tq (list torque))
            (cdp (list (float-vector 0 0 0)))
            (cdr (list (float-vector 0 0 0)))
            ;;(et (list extra))
            )
        (dotimes (i (1- average))
          (unix::usleep (round (* (/ 1.0 hz) 1000 1000)))
          (send *ri* :state)
          (push (send *ri* :potentio-vector) pt)
          ;;(push (send *ri* :torque-vector) tq)
          (let ((c (send cds :transformation (send (send *ri* :robot) :worldcoords))))
            (push (send c :pos) cdp)
            (push (coerce (car (send c :rpy-angle)) float-vector) cdr))
          #|(push
           (apply #'concatenate float-vector
                  (append (send *ri* :off-force-vector) (send *ri* :off-moment-vector)
                          (list (send *ri* :accel-vector))))
           et)|#
          )
        (setq potentio (vector-mean pt))
        ;;(setq torque (vector-mean tq))
        ;;(setq extra (vector-mean et))
        (setq cds (send cds :transform (make-coords :pos (vector-mean cdp) :rpy (vector-mean cdr))))
        ))
    (instance robot-state :init
              potentio
              (make-coords :rpy (car (send cds :rpy-angle)))
              ;; :torque torque
              ;; :extra extra
              :weight weight)
    ))

(defun transit-to-state (s-node)
  (let ((av
         (funcall (read-from-string (string-upcase (format nil "~A-pose" (send s-node :name)))))))
    (change-mode-for-getup-action)
    (send *ri* :angle-vector av 4000)
    (send *ri* :wait-interpolation)
    ))

(defun rolling-over-state-action ()
  (change-mode-for-getup-action)
  (face-up-to-face-down-action :real t :base-time 4200)
  (send *ri* :wait-interpolation)
  t
  )
(defun face-down-to-kneeling-state-action ()
  (change-mode-for-getup-action)
  (face-down-to-kneeling-action :real t :base-time 4200)
  (send *ri* :wait-interpolation)
  t
  )
(defun kneeling-to-sitting-state-action ()
  (change-mode-for-getup-action)
  (kneeling-to-sitting-action :real t :base-time 4200)
  (send *ri* :wait-interpolation)
  t
  )
(defun sitting-to-standing-state-action ()
  (sitting-to-standing-action :real t :base-time 4200)
  (send *ri* :wait-interpolation)
  t
  )

(defun build-statenet-jvrc (&key (robot *robot*))
  (setq *net* (instance state-net :init robot))

  (setq *default-weight*
        (let ((len (length (send *robot* :angle-vector))))
          (concatenate float-vector
                       (make-array len :element-type float-vector :initial-element 0.1)
                       (make-array 2   :element-type float-vector :initial-element 90))))
  ;; define states
  (setq *face-up-state*
        (instance robot-state :init
                  (face-up-pose)
                  (make-coords :rpy (list 0 -pi/2 0))
                  :weight *default-weight*))
  (setq *face-down-state*
        (instance robot-state :init
                  (face-down-pose)
                  (make-coords :rpy (list 0 pi/2 0))
                  :weight *default-weight*))
  (setq *kneeling-state*
        (instance robot-state :init
                  (kneeling-pose)
                  (make-coords :rpy (list 0 0.333 0))
                  :weight *default-weight*))
  (setq *sitting-state*
        (instance robot-state :init
                  (sitting-pose)
                  (make-coords :rpy (list 0 0 0))
                  :weight *default-weight*))
  (setq *standing-state*
        (instance robot-state :init
                  (standing-pose)
                  (make-coords :rpy (list 0 0 0))
                  :weight *default-weight*))
  ;; define nodes
  (let ((n-faceup (instance state-node :init robot *face-up-state*   :name "face-up"   :threshold 50.0))
        (n-facedn (instance state-node :init robot *face-down-state* :name "face-down" :threshold 50.0))
        (n-kneelg (instance state-node :init robot *kneeling-state*  :name "kneeling"  :threshold 30.0))
        (n-sittng (instance state-node :init robot *sitting-state*   :name "sitting"   :threshold 30.0))
        (n-standg (instance state-node :init robot *standing-state*  :name "standing"  :threshold 30.0))
        )
    (send *net* :add-node n-faceup)
    (send *net* :add-node n-facedn)
    (send *net* :add-node n-kneelg)
    (send *net* :add-node n-sittng)
    (send *net* :add-node n-standg)
    )

  (send *net* :add-arc-by-name "face-up" "face-down"
        :name "rolling-over" :cost 10)
  (send *net* :add-arc-by-name "face-down" "kneeling"
        :name "face-down-to-kneeling" :cost 10)
  (send *net* :add-arc-by-name "kneeling" "sitting"
        :name "kneeling-to-sitting" :cost 10)
  (send *net* :add-arc-by-name "sitting" "standing"
        :name "sitting-to-standing" :cost 10)

  *net*
  )

(defun start-statenet-demo ()
  (jaxon_jvrc-init)
  (build-statenet-jvrc)
  (warn "~%;;; start statenet ;;;~%")
  (while t
    (let ((cur (send *net* :find-nearest-state (sense-current-state :mode :check :average 10 :hz 40))))
      (unless (string= (send cur :name) "standing")
        (change-mode-for-getup-action)

        (send *net* :current-state cur)
        (send *net* :go-state "standing")
        )))
  )
#|
(send *net* :write-to-pdf "jvrc-statenet.pdf")
|#
