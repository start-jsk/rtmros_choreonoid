(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(require :choreonoid-service-client "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
(require :jvrc-action-utils "jvrc-action-utils.l")

(defun check-falling-down ()
  (send *ri* :state)
  (let* ((fv (send *ri* :force-vector))
         (rf (car fv))
         (lf (cadr fv))
         standingp direction)
    (when (> (+ (abs (elt rf 2))
                (abs (elt lf 2)))
             700)
      (setq standingp t))

    (let ((val (v. (float-vector 0 0 1)
                   (send *ri* :robot :rotate-vector (float-vector 1 0 0)))))
      (cond
       ((> val 0.4)
        (setq direction :face-up))
       ((< val -0.4)
        (setq direction :face-down))
       ))
    (if standingp
        nil
      direction)
    ))

(defun change-mode-for-getup-action ()
  (send *ri* :start-grasp)
  (send *ri* :stop-impedance :arms)
  ;; sometimes st does not stop by calling :stop-st
  (while (/= 0 (send (send *ri* :get-st-param) :controller_mode))
    ;;(print 'stop-st)
    (send *ri* :stop-st)
    (unix::usleep (* 200 1000)))
  (send *ri* :stop-auto-balancer)
  )

(defun get-up (&key (real t) (base-time 4000))
  (let ((falldir (check-falling-down)))
    (when falldir
      (change-mode-for-getup-action)

      (when (eq falldir :face-up)
        (face-up-to-face-down-action
         :real real :base-time base-time)
        )
      ;;
      (face-down-to-kneeling-action
       :real real :base-time base-time)
      ;;
      (kneeling-to-sitting-action
       :real real :base-time base-time)
      ;;
      (sitting-to-standing-action
       :real real :base-time base-time)
      ;;
      (send *ri* :wait-interpolation)
      )
    ))

(defun sitting-to-kneeling-pose-sequence ()
  (sitting-pose)
  (let (avs tms)
    (push (send *robot* :angle-vector) avs) ;; sitting-pose
    ;;
    (let ((rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          )
      (send *robot* :legs :crotch-p :joint-angle -5 :relative t)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (let ((rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          )
      ;;(send *robot* :legs :move-end-pos #f(0 0 -30))
      (send *robot* :legs :crotch-p :joint-angle -5 :relative t)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (send *robot* :larm :move-end-pos #f(0 0 30) :world)
    (send *robot* :rarm :move-end-pos #f(0 0 30) :world)
    (let ((rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          )
      (send *robot* :legs :move-end-pos #f(0 0 -30))
      (send *robot* :fix-leg-to-coords (make-coords)) ;; first-time
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    (fix-end-coords-on-ground :rarm-end-coords :larm-end-coords :l-foot-tip-hard-point)
    (let ((rh (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          (lh (make-coords :pos (copy-seq (send *robot* :larm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          )
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (send *robot* :larm :move-end-pos #f(0 0 40) :world)
    (send *robot* :rarm :move-end-pos #f(0 0 40) :world)
    (let (
          (rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          (lf (send *robot* :lleg :end-coords :copy-worldcoords))
          )
      (send *robot* :torso :waist-p :joint-angle -3 :relative t)
      (send *robot* :legs :crotch-p :joint-angle 6 :relative t)
      (send *robot* :move-coords lf (send *robot* :lleg :end-coords :worldcoords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    (fix-end-coords-on-ground :rarm-end-coords :larm-end-coords :l-foot-tip-hard-point)
    (let ((rh (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          (lh (make-coords :pos (copy-seq (send *robot* :larm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          )
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (send *robot* :larm :move-end-pos #f(0 0 40) :world)
    (send *robot* :rarm :move-end-pos #f(0 0 40) :world)
    (let (
          (rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          (lf (send *robot* :lleg :end-coords :copy-worldcoords))
          )
      (send *robot* :torso :waist-p :joint-angle -3 :relative t)
      (send *robot* :legs :crotch-p :joint-angle 6 :relative t)
      (send *robot* :move-coords lf (send *robot* :lleg :end-coords :worldcoords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    (fix-end-coords-on-ground :rarm-end-coords :larm-end-coords :l-foot-tip-hard-point)
    (let ((rh (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          (lh (make-coords :pos (copy-seq (send *robot* :larm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          )
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (send *robot* :larm :move-end-pos #f(0 0 40) :world)
    (send *robot* :rarm :move-end-pos #f(0 0 40) :world)
    (let (
          (rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          (lf (send *robot* :lleg :end-coords :copy-worldcoords))
          )
      (send *robot* :torso :waist-p :joint-angle -3 :relative t)
      (send *robot* :legs :crotch-p :joint-angle 6 :relative t)
      (send *robot* :move-coords lf (send *robot* :lleg :end-coords :worldcoords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    (fix-end-coords-on-ground :rarm-end-coords :larm-end-coords :l-foot-tip-hard-point)
    (let ((rh (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          (lh (make-coords :pos (copy-seq (send *robot* :larm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          )
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)

    ;;
    (send *robot* :larm :move-end-pos #f(0 0 40) :world)
    (send *robot* :rarm :move-end-pos #f(0 0 40) :world)
    (let (
          (rh (send *robot* :rarm :end-coords :copy-worldcoords))
          (lh (send *robot* :larm :end-coords :copy-worldcoords))
          (lf (send *robot* :lleg :end-coords :copy-worldcoords))
          )
      (send *robot* :torso :waist-p :joint-angle -3 :relative t)
      (send *robot* :legs :crotch-p :joint-angle 6 :relative t)
      (send *robot* :move-coords lf (send *robot* :lleg :end-coords :worldcoords))
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    (fix-end-coords-on-ground :rarm-end-coords :larm-end-coords :l-foot-tip-hard-point)
    (let ((rh (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          (lh (make-coords :pos (copy-seq (send *robot* :larm :end-coords :worldpos))
                           :rpy (list 0 pi/2 0)))
          )
      (send *robot* :rarm :inverse-kinematics rh)
      (send *robot* :larm :inverse-kinematics lh)
      )
    ;;(send-pose :real real :base-time base-time)
    (push (send *robot* :angle-vector) avs)
    ;; kneeling-pose
    (nreverse avs)
    ))

(defun kneeling-pose ()
  (send *robot* :angle-vector
        #f(0.010312 -0.002554 -59.9092 150.342 -76.4322 0.007123 -0.010312 0.002554 -59.9092 150.342 -76.4322 -0.007123 0.0 21.0 0.0 0.0 0.0 7.3323 -18.8803 -18.9175 8.81368 -23.1161 -21.0488 8.9624 0.918331 -7.31661 -18.8727 18.9135 -8.81477 -23.1471 21.0387 -8.95049 0.937098))
  (send *robot* :newcoords #s(coordinates plist nil rot #2f((0.929682 -2.600208e-05 0.368364) (-2.590045e-06 1.0 7.712488e-05) (-0.368364 -7.265567e-05 0.929682)) pos #f(211.741 -0.069351 -491.594)))
  (send *robot* :angle-vector)
  )

(defun sitting-pose (&key (move-hand t))
  (send *robot* :angle-vector
        #f(0.01017 -0.003347 -76.0 155.0 -79.0 0.007924 -0.01017 0.003347 -76.0 155.0 -79.0 -0.007924 0.0 33.0 0.0 0.0 0.0 0.0 -34.0 -15.8251 0.0 0.0 0.0 0.0 0.0 0.0 -34.0 15.8251 0.0 0.0 0.0 0.0 0.0))
  (send *robot* :fix-leg-to-coords (make-coords))
  (when move-hand
    (let ((lcds (make-coords :pos #f(430  350 0) :rpy (list 0 pi/2 0)))
          (rcds (make-coords :pos #f(430 -350 0) :rpy (list 0 pi/2 0))))
      (send *robot* :larm :inverse-kinematics lcds)
      (send *robot* :rarm :inverse-kinematics rcds)
      ))
  (send *robot* :angle-vector)
  )

(defun face-down-pose ()
  (send *robot* :init-pose)
  ;; set coords
  (send *robot* :angle-vector)
  )

(defun face-up-pose ()
  (send *robot* :init-pose)
  ;; set coords
  (send *robot* :angle-vector)
  )

(defun standing-pose ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  (send *robot* :angle-vector)
  )

(defun face-up-to-face-down-action
  (&key (real nil) (base-time 5000))
  (setq base-time (/ base-time 2))
  (send *robot* :init-pose)
  (when real
    (let ((diff
           (norm (v- (send *robot* :init-pose)
                     (send *ri* :state :potentio-vector)))))
      (setq diff (* diff 25))
      (if (< diff 300) (setq diff 300))
      (send-pose :real real :base-time diff)
      ))

  (send *robot* :rarm :collar-y :joint-angle 80)
  (send *robot* :larm :shoulder-p :joint-angle -90)
  (send *robot* :lleg :crotch-p :joint-angle -70)
  (send *robot* :lleg :crotch-y :joint-angle -15)
  (send-pose :real real :base-time base-time)

  (send *robot* :rleg :knee-p :joint-angle 90)
  (send *robot* :rleg :crotch-y :joint-angle -49)
  (send *robot* :lleg :knee-p :joint-angle 90);; ??
  (send-pose :real real :base-time base-time)

  (send *robot* :rleg :crotch-y :joint-angle 49)
  (send-pose :real real :base-time base-time)

  (send *robot* :larm :shoulder-p :joint-angle -90)
  (send *robot* :rarm :collar-y :joint-angle 0)
  (send *robot* :lleg :knee-p :joint-angle 0) ;; ??
  (send *robot* :rarm :shoulder-p :joint-angle 45)
  ;;(send *robot* :rarm :elbow-p :joint-angle 40)
  (send-pose :real real :base-time base-time)

  (send *robot* :init-pose)
  (send-pose :real real :base-time base-time)
  )

(defun sitting-to-standing-action
  (&key (real) (base-time 5000))
  (sitting-pose)

  (send *robot* :rarm :move-end-pos #f(0 0 30) :world)
  (mirror-angle-vector :limbs (list :rarm))
  (send-pose :real real :base-time (/ base-time 3))

  (send *robot* :reset-pose)
  (let ((r-reset (send *robot* :rarm :angle-vector)))
    (sitting-pose)
    (send *robot* :rarm :angle-vector r-reset)
    (mirror-angle-vector :limbs (list :rarm))
    )
  (send-pose :real real :base-time (/ base-time 3))

  (send *robot* :legs :move-end-pos #f(0 0 -50))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  (send-pose :real real :base-time (/ base-time 2))

  ;;
  (when real
   (send *ri* :kalmanfilterservice_resetkalmanfilterstate)
   (send *ri* :start-auto-balancer)
   (send *ri* :start-st)
   )

  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  (send-pose :real real :base-time base-time)
  )

(defun kneeling-to-sitting-action
  (&key (real) (base-time 5000))
  (let* ((avs (nreverse (sitting-to-kneeling-pose-sequence)))
         (tms (make-list (length avs) :initial-element (/ base-time (length avs))))
         )
    (when real
      (send *ri* :wait-interpolation)
      (send *ri* :angle-vector-sequence avs tms))
    )
  )

(defun face-down-to-kneeling-action
  (&key (real) (base-time 5000))
  ;;
  (let (avs)
  (send *robot* :init-pose)
  (send *robot* :rleg :crotch-p :joint-angle  -10)
  (send *robot* :rleg :knee-p   :joint-angle   20)
  (send *robot* :rleg :ankle-p  :joint-angle -40)
  (send *robot* :rarm :shoulder-p :joint-angle  85)
  (send *robot* :rarm :shoulder-r :joint-angle -40)
  (send *robot* :rarm :elbow-p    :joint-angle -85)
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  (send *robot* :move-coords (make-coords :rpy (list 0 pi/2 0)) (send *robot* :body_lk))
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos (float-vector 140 -460 -60) :rpy (list pi pi/2 pi/2)))
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  (send *robot* :rarm :move-end-pos (float-vector 120 0 0))
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  (send *robot* :rarm :move-end-pos (float-vector 240 0 0))
  (let ((f (v- (send *robot* :rarm :end-coords :worldpos)
               (send *robot* :rleg :end-coords :worldpos))))
    (send *robot* :rotate (atan2 (elt f 2) (elt f 0)) :y))
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                     :rpy (list pi pi/2 pi/2)))
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  (send *robot* :rarm :move-end-pos (float-vector 0 0 -140) :world)
  (let ((f (v- (send *robot* :rarm :end-coords :worldpos)
               (send *robot* :rleg :end-coords :worldpos))))
    (send *robot* :rotate (atan2 (elt f 2) (elt f 0)) :y))
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos (copy-seq (send *robot* :rarm :end-coords :worldpos))
                     :rpy (list pi pi/2 pi/2)))
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  (move-base-pose (make-coords :pos (float-vector -200 0 0)) :wrt :world
             :rotation-axis (list nil nil t t) :fullbody nil)
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  ;;
  ;;(move-limbs :rlimb :rarm :llimb :larm :forward 0 :side 150 :real real :base-time base-time)
  (send *robot* :rarm :move-end-pos #f(0 150 0) :world)
  (send *robot* :rleg :knee-p :joint-angle 12 :relative t)
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  (send *robot* :rleg :crotch-p :joint-angle -111)
  (send *robot* :rleg :knee-p   :joint-angle  155)
  (send *robot* :rleg :ankle-p :joint-angle   -79)
  (mirror-angle-vector :limbs (list :rleg :rarm))
  ;;(send-pose :real real :base-time (/ base-time 2))
  (push (send *robot* :angle-vector) avs)

  (setq avs (nreverse avs))
  (when real
    (let* ((tms (make-list (length avs) :initial-element (/ (* 2 base-time) (length avs))))
           )
      (send *ri* :wait-interpolation)
      (send *ri* :angle-vector-sequence avs tms)
      ))
  ;;
  (kneeling-pose)
  (send-pose :real real :base-time (/ base-time 2))
  (setq avs (nreverse avs))
  (push (send *robot* :angle-vector) avs)
  (nreverse avs)
  ))

#|
(jaxon_jvrc-init)
(setq *robot* *jaxon_jvrc*)
(send *ri* :stop-impedance :arms)
(send *ri* :stop-st)
(send *ri* :stop-auto-balancer)
(send *ri* :start-grasp)
(warn "~%~%;;;; start ;;;;~%")

(load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l") ;; for simulation
|#
