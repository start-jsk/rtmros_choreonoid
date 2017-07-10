(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")

(require :jvrc-standup "jvrc-standup.l")
(require :jvrc-kick-motion "jvrc-kick-motion.l")
(require :walk-to-target-lib "walk-to-target-lib.l")
(require :jvec-statenet "jvrc-statenet.l")

(initialize-robot-for-walk-to-target)
(setup-kick-motion :initialize-robot-pose nil)
(build-statenet-jvrc)
(setup-walk-to-target)

(defun reset-to-default-pose (&key (real t) (tm 1400))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  (when real (send *ri* :angle-vector (send *robot* :angle-vector) tm))
  )

(defun standup-with-statenet ()
  (let ((cur (send *net* :find-nearest-state
                   (sense-current-state :mode :check :average 10 :hz 40))))
    (unless (string= (send cur :name) "standing")
      (change-mode-for-getup-action)
      (send *net* :current-state cur)
      (send *net* :go-state "standing")
      ;;
      (send *ri* :wait-interpolation)
      (reset-to-default-pose)
      ;;
      (send *ri* :wait-interpolation)
      )))

(defun kick-ball (leg speed)
  (change-to-single-leg-mode leg :leg-height 50)
  (kick-motion :swing-speed speed :kick-crotch-p -72)
  (change-to-double-leg-mode)
  )

(defun kick-ball-demo ()
  (while t
    ;;(warn "~%;;; start walking to target ;;;~%~%")
    (let ((ret (walk-to-target :stop-threshold 600)))
      (warn "~%;;; stop walking (~A) ;;;~%~%" ret)
      (case (car ret)
        (:find-target
         (warn "~%;;; kick motion ;;;~%~%")
         (kick-ball :lleg 700)
         (standup-with-statenet)
         )
        (t ;; :lost-target
         (standup-with-statenet)
         (send *ri* :go-pos -0.7 0 0)
         (standup-with-statenet)
         (warn "~%;;; search target ;;;~%~%")
         (let ((ret (search-target)))
           (unless ret
             (send *ri* :go-pos 0 0 120)
             ))
         (standup-with-statenet)
         )
        ))
    (reset-to-default-pose)
    (send *ri* :wait-interpolation)
    ))

;;
;;
;;
(defun kick-ball-to (ball-pos goal-pos &key (real t) (leg :rleg))
  (let (th dir t-dir robot-pos)
    (setq dir (v- goal-pos ball-pos))
    (setf (elt dir 2) 0)
    (setq dir (normalize-vector dir dir))
    (setq t-dir (v* #f(0 0 1) dir))

    (setq robot-pos (v+ ball-pos (scale -450 dir)))    ;; 450 offset for kick
    (setq robot-pos (v+ robot-pos (scale 110 t-dir))) ;; offset for rleg

    (setq th (rad2deg (atan (elt dir 1) (elt dir 0))))
    (pprint (list dir robot-pos th))
    ;;
    (when real
      (when (> (abs th) 40)
        (send *ri* :go-pos -0.4 0 0)
        (v+ robot-pos (float-vector 400 0 0) robot-pos))
      (send *ri* :go-pos
            (/ (elt robot-pos 0) 1000)
            (/ (elt robot-pos 1) 1000)
            th)
      (kick-ball leg 700)
      )
    ))

(defun ball-is-goalq ()
  (let ((pos (send (get-coords-on-simulation :robot "BALL") :worldpos)))
    (and (> (elt pos 0) 9600)
         (> 1560 (elt pos 1) -1560)
         (> 2200 (elt pos 2) 0))
    ))

(defun goal-motion ()
  (send *robot* :reset-pose)
  (send *robot* :rarm :shoulder-p :joint-angle -120)
  (send *robot* :rarm :elbow-p :joint-angle -60)
  (send *robot* :rarm :wrist-p :joint-angle 0 )
  (send *robot* :rarm :wrist-r :joint-angle 0 )
  (send *robot* :rarm :wrist-y :joint-angle 0 )
  ;;
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  ;;
  (send *ri* :angle-vector (send *robot* :angle-vector) 1200)
  (send *ri* :wait-interpolation)
  )

(defun kick-ball-to-goal-demo ()
  (while t
    (when (ball-is-goalq)
      (warn "~%;;; GOOOOOOAAAAAAL ;;;~%~%")
      (goal-motion)
      (return-from kick-ball-to-goal-demo))
    ;; (warn "~%;;; start walking to target ;;;~%~%")
    (let ((ret (walk-to-target :stop-threshold 600
                               :finish-query-func #'ball-is-goalq
                               )))
      (warn "~%;;; stop walking (~A) ;;;~%~%" ret)
      (cond
       ((eq :find-target (car ret))
        (warn "~%;;; kick motion ;;;~%~%")
        ;;
        (let ((goal-cds (get-coords-on-simulation :robot "GOAL" :link "BASE"))
              (robot-cds (get-coords-on-simulation :robot "JAXON_RED")))
          ;;
          (kick-ball-to (detect-target)
                        (send (send robot-cds :transformation goal-cds) :worldpos))
          ;;
          (standup-with-statenet)
          ))
       ((eq :finish-query (car ret))
        (warn "~%;;; GOOOOOOAAAAAAL ;;;~%~%")
        (goal-motion)
        (return-from kick-ball-to-goal-demo)
        )
       (t ;; :lost-target
        (standup-with-statenet)
        (send *ri* :go-pos -0.7 0 0)
        (standup-with-statenet)
        (warn "~%;;; search target ;;;~%~%")
        (let ((ret (search-target)))
          (unless ret
            (send *ri* :go-pos 0 0 120)
            ))
        (standup-with-statenet)
        )
       ))
    (warn "~%;;; reset pose ;;;~%~%")
    (reset-to-default-pose)
    (send *ri* :wait-interpolation)
    ))

(defun make-random-ball-position (&key (x-range 5000.0) (y-range 10000.0))
  (setq *random-state* (make-random-state))
  (let ((cds
         (make-coords :pos (float-vector (+ 1000 (random x-range))
                                         (- (random y-range) (* 0.5 y-range))
                                         200)
                      )))
    (reset-simulation :coords cds :robot "BALL")
    cds
    ))

(warn "
(make-random-first-position)
(make-random-ball-position)
(kick-ball-demo)
(kick-ball-to-goal-demo)
")
