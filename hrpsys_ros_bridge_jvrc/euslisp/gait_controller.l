#!/usr/bin/env roseus

(load "package://pr2eus/speak.l")                       ; for speak-en
(load "package://drc_task_common/euslisp/robot-util.l") ; for robot-file

(defclass quadruped-util)
(defmethod quadruped-util
  (:init ())
  (:check-ready
   ()
   (speak-en "checking if i am ready" :timeout 1)
   (send *ri* :start-grasp)
   (let ((abc-mode (send *ri* :get-auto-balancer-controller-mode))
         (st-mode (send *ri* :get-st-controller-mode))
         (ic-mode-arms (send *ri* :get-impedance-controller-controller-mode :arms))
         (hand-av (if (find-method *ri* :get-joint-angle) (send *ri* :get-joint-angle))))
     (warning-message 2 (format nil "[~A] ABC mode : ~A (~A)~%" (ros::get-name) (eq :mode_abc abc-mode) abc-mode))
     (warning-message 2 (format nil "[~A]  ST mode : ~A (~A)~%" (ros::get-name) (eq :mode_st st-mode) st-mode))
     (warning-message 2 (format nil "[~A]  IC mode : ~A (~A)~%" (ros::get-name) (mapcar #'(lambda (x) (eq :mode_idle x)) ic-mode-arms) ic-mode-arms))
     (if hand-av
         (warning-message 2 (format nil "[~A] hand : ~A (~A)~%" (ros::get-name) (mapcar #'(lambda (a) (> (elt a 0) 0.95)) hand-av) hand-av)))
     (let ((ans
            (and (eq :mode_abc abc-mode) (eq :mode_st st-mode)
                 (eq :mode_idle (car ic-mode-arms)) (eq :mode_idle (cadr ic-mode-arms))
                 (if hand-av (and (> (elt (car hand-av) 0) 0.95) (> (elt (cadr hand-av) 0) 0.95))
                   t))))
       (if ans
           (speak-en "ok ok ok ok ok ok" :timeout 1)
         (speak-en "not yet not yet" :timeout 1))
       ans))
   )
  (:cc-converter
   (fssl)
   (let* ((orig-list (mapcar #'(lambda (l) (send (send *robot* l :end-coords) :copy-worldcoords)) (list :rarm :larm)))
          (dst-list (mapcar #'(lambda (l) (send (send *robot* :get l) :copy-worldcoords)) (list :rhand-contact-coords :lhand-contact-coords)))
          (dst-from-orig-list (mapcar #'(lambda (o d) (send o :transformation d :local)) orig-list dst-list))
          (new-fssl nil))
     (dolist (fss fssl)
       (let (new-fss)
         (dolist (fs fss)
           (let ((l (send fs :name)))
             (cond ((string-equal ':rarm l)
                    (setq fs (send (send (car dst-from-orig-list) :copy-worldcoords) :transform fs :world))
                    (send fs :name :rarm))
                   ((string-equal ':larm l)
                    (setq fs (send (send (cadr dst-from-orig-list) :copy-worldcoords) :transform fs :world))
                    (send fs :name :larm)
                    )))
           (push fs new-fss))
         (push new-fss new-fssl)))
     (reverse new-fssl)))
  (:get-xy-footstep
   (x y &key
      (forward-offset-length 150) ; one step x limit
      (outside-offset-length 100)  ; one step y limit
      (type :crawl))
   (let* ((div (max (floor (/ (abs x) forward-offset-length))
                    (floor (/ (abs y) outside-offset-length))))
          (x-step (/ x (+ div 1)))
          (y-step (/ y (+ div 1)))
          ;; the position = (equal) is very important
          (order (cond ((eq type :crawl)
                        (cond ((and (> x 0) (> y 0))
                               (list (list :lleg) (list :rleg) (list :rarm) (list :larm)))
                              ((and (> x 0) (<= y 0))
                               (list (list :rleg) (list :lleg) (list :larm) (list :rarm)))
                              ((and (<= x 0) (> y 0))
                               (list (list :rarm) (list :larm) (list :lleg) (list :rleg)))
                              ((and (<= x 0) (<= y 0))
                               (list (list :larm) (list :rarm) (list :rleg) (list :lleg)))))
                       ((eq type :trot)
                        (cond ((> y 0)
                               (list (list :lleg :rarm) (list :rleg :larm)))
                              ((<= y 0)
                               (list (list :rleg :larm) (list :lleg :rarm)))))))
          (fs-list))
     ;; dummy support legs
     (push (mapcar #'(lambda (l) (make-coords :coords (send *robot* l :end-coords :copy-worldcoords) :name l))
                   (car (last order)))
           fs-list)
     ;; main
     (dotimes (i div)
       (dolist (ll order)
         (push (mapcar #'(lambda (l)
                           (make-coords
                            :coords (send (send *robot* l :end-coords :copy-worldcoords)
                                          :translate (float-vector (* (+ i 1) x-step) (* (+ i 1) y-step) 0) :world)
                            :name l))
                       ll)
               fs-list)
         ))
     ;; residual
     (dolist (ll order)
       (push (mapcar #'(lambda (l)
                         (make-coords
                          :coords (send (send *robot* l :end-coords :copy-worldcoords)
                                        :translate (float-vector x y 0) :world)
                          :name l))
                     ll)
             fs-list)
       )
     (reverse fs-list)))
  (:get-yaw-footstep
   (yaw &key
        (rotate-rad 20) ; one step th limit
        (type :crawl))
   (let* ((div (floor (/ (abs yaw) rotate-rad)))
          (th-step (/ yaw (+ div 1)))
          (center (make-coords :pos (send *robot* :centroid)))
          (order (cond ((eq type :crawl)
                        (cond ((> yaw 0)
                               (list (list :rarm) (list :rleg) (list :larm) (list :lleg)))
                              ((<= yaw 0)
                               (list (list :larm) (list :lleg) (list :rarm) (list :rleg)))))
                       ((eq type :trot)
                        (cond ((> yaw 0)
                               (list (list :larm :rleg) (list :rarm :lleg)))
                              ((<= yaw 0)
                               (list (list :rarm :lleg) (list :larm :rleg)))))))
          (ox-all (mapcar #'(lambda (ll)
                              (mapcar #'(lambda (l)
                                          (send (send center :copy-worldcoords) :transformation (send *robot* l :end-coords :copy-worldcoords)))
                                      ll))
                          order))
          (fs-list))
     ;; dummy support legs
     (push (mapcar #'(lambda (l) (make-coords :coords (send *robot* l :end-coords :copy-worldcoords) :name l))
                   (car (last order)))
           fs-list)
     ;; main
     (let (new-center)
       (dotimes (i div)
         (setq new-center (send (send center :copy-worldcoords) :rotate (deg2rad (* (+ i 1) th-step)) :z :local))
         (dotimes (j (length order))
           (push (mapcar #'(lambda (l ox)
                             (make-coords
                              :coords (send (send ox :copy-worldcoords) :transform (send new-center :copy-worldcoords) :world)
                              :name l))
                         (elt order j) (elt ox-all j))
                 fs-list)
           )))
     ;; residual
     (let ((new-center (send (send center :copy-worldcoords) :rotate (deg2rad yaw) :z :local)))
       (dotimes (j (length order))
         (push (mapcar #'(lambda (l ox)
                           (make-coords
                            :coords (send (send ox :copy-worldcoords) :transform (send new-center :copy-worldcoords) :world)
                            :name l))
                       (elt order j) (elt ox-all j))
               fs-list)
         ))
     (reverse fs-list)))
  (:release-inner-force
   (type)
   (speak-en "release inner force" :timeout 1)
   (cond ((eq type 'biped) (send *ri* :go-pos 0 0 0))
         ((eq type 'quadruped)
          (send *robot* :angle-vector (send *ri* :state :potentio-vector))
          (send *robot* :fix-leg-to-coords (make-coords))
          (send *ri* :set-foot-steps (send self :cc-converter (send self :get-xy-footstep 0 0 :type :crawl))))
         ))
  ;; (objects (list *robot* (get-bounding-box-of-links (send *robot* :links))))
  (:get-bounding-box-of-links
   (links)
   (let* ((bodies (flatten (send-all (send *robot* :links) :bodies)))
          (vertices (flatten (send-all bodies :vertices)))
          (bbox (make-bounding-box vertices))
          (bbox-obj (send bbox :body)))
     (print (v- (send bbox :maxpoint) (send bbox :minpoint)))
     (setf (get bbox-obj :face-color) :green)
     (gl::transparent bbox-obj 0.2)
     bbox-obj))
  )


(defclass multi-gait
  :super propertied-object
  :slots (robot-name use-viewer
                     biped-pose touch-down-pose quadruped-pose
                     biped-params quadruped-params
                     q-util
                     current-gait-type)
  )

(defmethod multi-gait
  (:init
   (&key (robot 'jaxon_jvrc) (make-robot-model t))
   (setq robot-name robot
         use-viewer (ros::get-param "~use_viewer" nil)
         q-util (instance quadruped-util :init)
         current-gait-type 'biped)
   (when make-robot-model
     (load (robot-file robot))
     (setq *robot* (init-robot-from-name robot)))
   (if use-viewer (objects (list *robot*)))
   ;; set biped-pose
   (progn
     (send *robot* :reset-pose)
     (send *robot* :fix-leg-to-coords (make-coords))
     (send *robot* :move-centroid-on-foot :both '(:rarm :larm :rleg :lleg))
     (setq biped-pose (send *robot* :angle-vector)))
   ;; set touch-down-pose
   (progn
     (send *robot* :angle-vector biped-pose)
     (send *robot* :fix-leg-to-coords (make-coords))
     (let ((tc (list
                (send *robot* :rleg :end-coords :copy-worldcoords)
                (send *robot* :lleg :end-coords :copy-worldcoords)
                (make-coords :pos (v+ (send *robot* :rleg :end-coords :worldpos) (float-vector 400 -200 0)))
                (make-coords :pos (v+ (send *robot* :lleg :end-coords :worldpos) (float-vector 400 +200 0)))))
           (mt (list (send *robot* :rleg :end-coords)
                     (send *robot* :lleg :end-coords)
                     (send *robot* :get :rhand-contact-coords)
                     (send *robot* :get :lhand-contact-coords))))
       (send *robot*
             :fullbody-inverse-kinematics tc
             :move-target mt
             :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send x :parent))) mt)
             :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                           (list (send *robot* :rarm :collar-y :child-link) 0.8)
                                           (list (send *robot* :larm :collar-y :child-link) 0.8))
             :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
             :min (float-vector -100 -100 -1000 -90 -90 -90)
             :max (float-vector +100 +100 +1000 +90 +90 +90)
             :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.8 0.1 0.8 0.0)
             :debug-view :no-message))
     (setq touch-down-pose (send *robot* :angle-vector)))
   ;; set quadruped-pose
   (progn
     (send *robot* :angle-vector touch-down-pose)
     (send *robot* :fix-leg-to-coords (make-coords))
     (let ((tc (list (send *robot* :rleg :end-coords :copy-worldcoords)
                     (send *robot* :lleg :end-coords :copy-worldcoords)
                     (send (send *robot* :get :rhand-contact-coords) :copy-worldcoords)
                     (send (send *robot* :get :lhand-contact-coords) :copy-worldcoords)
                     (send (send (apply #'midcoords 0.5 (send *robot* :legs :end-coords)) :copy-worldcoords) :translate  #f(0 0 480) :world)))
           (mt (list (send *robot* :rleg :end-coords)
                     (send *robot* :lleg :end-coords)
                     (send *robot* :get :rhand-contact-coords)
                     (send *robot* :get :lhand-contact-coords)
                     (send *robot* :get :hip-contact-coords))))
       (send *robot* :legs  :knee-p   :joint-angle 80)
       (send *robot* :legs  :crotch-p :joint-angle -80)
       (send *robot* :torso :waist-p  :joint-angle 30)
       (send *robot* :arms :elbow-p :joint-angle -40) ; for natual elbow bending
       (send *robot* :head :neck-p :joint-angle -20)
       (send *robot* :fix-leg-to-coords (make-coords))
       (send *robot*
             :fullbody-inverse-kinematics tc
             :move-target mt
             :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send x :parent))) mt)
             :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                           (list (send *robot* :rarm :collar-y :child-link) 0.8)
                                           (list (send *robot* :larm :collar-y :child-link) 0.8))
             :target-centroid-pos (scale (/ 1.0 4) (reduce #'v+ (send-all (subseq tc 0 4) :worldpos)))
             :translation-axis (list t t t t :x)
             :rotation-axis (list t t t t :y)
             :min (float-vector -100 -100 -1000 -90 -90 -90)
             :max (float-vector +100 +100 +1000 +90 +90 +90)
             :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.8 0.1 0.8 0.1)
             :debug-view :no-message)
       (setq quadruped-pose (send *robot* :angle-vector))))
   ;; store biped-params
   (let ((current-st-param (send *ri* :get-st-param))
         (current-gg-param (send *ri* :get-gait-generator-param)))
     (setq biped-params
           (list :auto-balancer-param (list :leg-names (list :rleg :lleg)
                                            :default-gait-type 0 ;BIPED
                                            :end-effector-list (let ((abc-ee (send (send *ri* :get-auto-balancer-param) :end_effector_list)))
                                                                 (dolist (ee abc-ee)
                                                                   (let ((tmpcoords (send (send *robot* (intern (string-upcase (send ee :leg)) :keyword) :end-coords) :coords)))
                                                                     (send ee :pos (scale 1e-3 (send tmpcoords :pos)))
                                                                     (send ee :rot (user::matrix2quaternion (send tmpcoords :rot)))))
                                                                 abc-ee))
                 :gait-generator-param (list :default-step-time (send current-gg-param :default_step_time)
                                             :default-double-support-ratio (send current-gg-param :default_double_support_ratio)
                                             :default-step-height (send current-gg-param :default_step_height)
                                             :default-orbit-type (send current-gg-param :default_orbit_type)
                                             :stride-parameter (send current-gg-param :stride_parameter))
                 :st-param (list :eefm-rot-damping-gain (let ((current-eefm-rot-damping-gain (send (send current-st-param :eefm_rot_damping_gain) :data)))
                                                          (mapcar #'(lambda (i) (subseq current-eefm-rot-damping-gain (* i 3) (* (+ i 1) 3)))
                                                                  (list 0 1 2 3)))
                                 :eefm-pos-damping-gain (let ((current-eefm-pos-damping-gain (send (send current-st-param :eefm_pos_damping_gain) :data)))
                                                          (mapcar #'(lambda (i) (subseq current-eefm-pos-damping-gain (* i 3) (* (+ i 1) 3)))
                                                                  (list 0 1 2 3)))
                                 :st-algorithm (send current-st-param :st_algorithm)
                                 :is-ik-enable (list t t nil nil)
                                 :is-feedback-control-enable (list t t nil nil)
                                 :is-zmp-calc-enable (list t t nil nil)
                                 :eefm-use-force-difference-control t
                                 :end-effector-list (let ((st-ee (send current-st-param :end_effector_list)))
                                                      (dolist (ee st-ee)
                                                        (let ((tmpcoords (send (send *robot* (intern (string-upcase (send ee :leg)) :keyword) :end-coords) :coords)))
                                                          (send ee :pos (scale 1e-3 (send tmpcoords :pos)))
                                                          (send ee :rot (user::matrix2quaternion (send tmpcoords :rot)))))
                                                      st-ee)))))
   ;; store quadruped-params
   (let ((biped-step-time (cadr (memq :default-step-time (cadr (memq :gait-generator-param biped-params)))))
         (biped-double-support-ratio (cadr (memq :default-double-support-ratio (cadr (memq :gait-generator-param biped-params)))))
         (speed-up-ratio 0.9))
     (send *robot* :angle-vector quadruped-pose)
     (send *robot* :fix-leg-to-coords (make-coords))
     (setq quadruped-params
           (list :auto-balancer-param (list :leg-names (list :rleg :lleg :rarm :larm)
                                            :default-gait-type 1 ;TROT
                                            :end-effector-list (let ((abc-ee (cadr (memq :end-effector-list (cadr (memq :auto-balancer-param biped-params))))))
                                                                 (dolist (ee abc-ee)
                                                                   (if (or (string-equal (send ee :leg) "rarm") (string-equal (send ee :leg) "larm"))
                                                                       (let ((tmpcoords (send (send *robot* :get
                                                                                                    (if (string-equal (send ee :leg) "rarm")
                                                                                                        :rhand-contact-coords
                                                                                                      :lhand-contact-coords)) :coords)))
                                                                         (send ee :pos (scale 1e-3 (send tmpcoords :pos)))
                                                                         (send ee :rot (user::matrix2quaternion (send tmpcoords :rot))))))
                                                                 abc-ee))
                 :gait-generator-param (list :default-step-time (* speed-up-ratio biped-step-time) ;speed up
                                             :default-double-support-ratio (let ((swing-time (* (- 1.0 biped-double-support-ratio) biped-step-time)))
                                                                             (- 1.0 (/ swing-time (* speed-up-ratio biped-step-time))))
                                             :default-step-height 0.08
                                             :default-orbit-type 2 ;RECTANGLE
                                             :stride-parameter (float-vector 0.15 0.10 20.0 0.15)
                                             :leg-default-translate-pos (list (send (send *robot* :rleg :end-coords) :worldpos)
                                                                              (send (send *robot* :lleg :end-coords) :worldpos)
                                                                              (send (send *robot* :get :rhand-contact-coords) :worldpos)
                                                                              (send (send *robot* :get :lhand-contact-coords) :worldpos))
                                             :zmp-weight-map (list 1 1 0.6 0.6))
                 :st-param (list :eefm-rot-damping-gain (mapcar #'(lambda (ratio gain)
                                                                    (float-vector (* (elt ratio 0) (elt gain 0))
                                                                                  (* (elt ratio 1) (elt gain 1))
                                                                                  (* (elt ratio 2) (elt gain 2))))
                                                                (list #f(0.72 0.72 1.0) ;rleg
                                                                      #f(0.72 0.72 1.0) ;lleg
                                                                      #f(1.00 1.00 1.0) ;rarm
                                                                      #f(1.00 1.00 1.0)) ;larm
                                                                (cadr (memq :eefm-rot-damping-gain (cadr (memq :st-param biped-params)))))
                                 :eefm-pos-damping-gain (mapcar #'(lambda (ratio gain)
                                                                    (float-vector (* (elt ratio 0) (elt gain 0))
                                                                                  (* (elt ratio 1) (elt gain 1))
                                                                                  (* (elt ratio 2) (elt gain 2))))
                                                                (list #f(5 5 1) ;rleg
                                                                      #f(5 5 1) ;lleg
                                                                      #f(5 5 1) ;rarm
                                                                      #f(5 5 1)) ;larm
                                                                (cadr (memq :eefm-pos-damping-gain (cadr (memq :st-param biped-params)))))
                                 :emergency-check-mode 0
                                 :st-algorithm 3 ;EEFMQPCOP
                                 :is-ik-enable (list t t t t)
                                 :is-feedback-control-enable (list t t t t)
                                 :is-zmp-calc-enable (list t t t t)
                                 :eefm-use-force-difference-control nil
                                 :end-effector-list (let ((st-ee (cadr (memq :end-effector-list (cadr (memq :st-param biped-params))))))
                                                      (dolist (ee st-ee)
                                                        (if (or (string-equal (send ee :leg) "rarm") (string-equal (send ee :leg) "larm"))
                                                            (let ((tmpcoords (send (send *robot* :get
                                                                                         (if (string-equal (send ee :leg) "rarm")
                                                                                             :rhand-contact-coords
                                                                                           :lhand-contact-coords)) :coords)))
                                                              (send ee :pos (scale 1e-3 (send tmpcoords :pos)))
                                                              (send ee :rot (user::matrix2quaternion (send tmpcoords :rot))))))
                                                      st-ee)))))
   )
  (:biped->quadruped
   ()
   "Assuming that abc and st is ON and paramters are set to default"
   (send *ri* :start-grasp)
   (send *ri* :stop-impedance :arms)    ;should not stop?
   (progn
     (unless (send q-util :check-ready)
       (return-from :biped->quadruped nil))
     (unless (eq current-gait-type 'biped)
       (warning-message 3 (format nil "[~A] cannot call :biped->quadruped in ~A mode~%" (ros::get-name) current-gait-type))
       (return-from :biped->quadruped nil)))
   ;; (send *ri* :reset-force-moment-offset-arms)
   (send *ri* :set-st-param :emergency-check-mode 0)
   (progn
     ;; RYOBI HOIST
     (send *ri* :angle-vector touch-down-pose 3000)
     ;; IREX HOIST
     ;; (send *ri* :angle-vector touch-down-pose 6000)
     (speak-en "u go e te ru" :timeout 1)
     (send *ri* :wait-interpolation))
   (progn
     (speak-en "es te ei b c to me te ru" :timeout 1)
     (send *ri* :stop-default-unstable-controllers))
   (progn
     (send *ri* :angle-vector quadruped-pose 3000)
     (speak-en "u go e te ru" :timeout 1)
     (send *ri* :wait-interpolation))
   (progn                               ;set parameters
     (send* *ri* :set-auto-balancer-param (cadr (memq :auto-balancer-param quadruped-params)))
     (send* *ri* :set-gait-generator-param (cadr (memq :gait-generator-param quadruped-params)))
     (send* *ri* :set-st-param (cadr (memq :st-param quadruped-params))))
   (progn
     (speak-en "ei b c e re te e ru" :timeout 1)
     (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
     (speak-en "es te e re te e ru" :timeout 1)
     (send *ri* :start-st))
   (setq current-gait-type 'quadruped)
   )
  (:quadruped->biped
   ()
   (progn
     (unless (eq current-gait-type 'quadruped)
       (warning-message 3 (format nil "[~A] cannot call :quadruped->biped in ~A mode~%" (ros::get-name) current-gait-type))
       (return-from :quadruped->biped nil)))
   (progn
     (speak-en "es te ei b c to me te ru" :timeout 1)
     (send *ri* :stop-default-unstable-controllers))
   (progn
     (send *ri* :angle-vector touch-down-pose 3000)
     (speak-en "u go e te ru" :timeout 1)
     (send *ri* :wait-interpolation))
   (progn                               ;set parameters
     (send* *ri* :set-auto-balancer-param (cadr (memq :auto-balancer-param biped-params)))
     (send* *ri* :set-gait-generator-param (cadr (memq :gait-generator-param biped-params)))
     (send* *ri* :set-st-param (cadr (memq :st-param biped-params))))
   (progn
     (speak-en "ei b c e re te ru" :timeout 1)
     (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
     (speak-en "es te e re te ru" :timeout 1)
     (send *ri* :start-st))
   (progn
     ;; RYOBI HOIST
     ;; (send *ri* :angle-vector biped-pose 2000)
     ;; IREX HOIST
     (send *ri* :angle-vector biped-pose 6000)
     (speak-en "u go e te ru" :timeout 1)
     (send *ri* :wait-interpolation))
   (send *ri* :set-st-param :emergency-check-mode 2)
   (setq current-gait-type 'biped)
   )
  )




;;;;;; memo
;;;; 1m mae
;; (send *ri* :set-foot-steps (cc-converter (get-xy-footstep 1000 0 :forward-offset-length 150 :outside-offset-length 100 :type :crawl)))
;;;; 10deg migi
;; (send *ri* :set-foot-steps (cc-converter (get-yaw-footstep -10 :rotate-rad 20 :type :crawl)))

#|
(check-ready)

(biped->quadruped)

(release-inner-force)

;; 1m mae
(send *ri* :set-foot-steps (cc-converter (get-xy-footstep 1000 0 :forward-offset-length 150 :outside-offset-length 100 :type :crawl)))
;; 40 cm
(send *ri* :set-foot-steps (cc-converter (get-xy-footstep 400 0 :forward-offset-length 150 :outside-offset-length 100 :type :crawl)))

;; Y migi 20cm
(send *ri* :set-foot-steps (cc-converter (get-xy-footstep 0 -200 :forward-offset-length 150 :outside-offset-length 100 :type :crawl)))

;; yaw
(send *ri* :set-foot-steps (cc-converter (get-yaw-footstep -10 :rotate-rad 20 :type :crawl)))

(quadruped->biped)
|#
