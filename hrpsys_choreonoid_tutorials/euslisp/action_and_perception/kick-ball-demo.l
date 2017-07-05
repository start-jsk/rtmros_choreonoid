(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(require :choreonoid-service-client "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")

(require :jvrc-standup "jvrc-standup.l")
(require :jvrc-kick-motion "jvrc-kick-motion.l")
(require :walk-to-target-lib "walk-to-target-lib.l")
(require :jvec-statenet "jvrc-statenet.l")

(initialize-robot-for-walk-to-target)
(setup-kick-motion :initialize-robot-pose nil)
(setup-walk-to-target)
(build-statenet-jvrc)

(defun standup-with-statenet ()
  (let ((cur (send *net* :find-nearest-state
                   (sense-current-state :mode :check :average 10 :hz 40))))
    (unless (string= (send cur :name) "standing")
      (change-mode-for-getup-action)
      (send *net* :current-state cur)
      (send *net* :go-state "standing")
      )))

(defun ball-kick-demo ()
  (while t
    ;;(warn "~%;;; start walking to target ;;;~%~%")
    (let ((ret (walk-to-target :stop-threshold 600)))
      (warn "~%;;; stop walking (~A) ;;;~%~%" ret)
      (case ret
        (:find-target
         (warn "~%;;; kick motion ;;;~%~%")
         (change-to-single-leg-mode :rleg)
         (kick-motion :swing-speed 700)
         (change-to-double-leg-mode)
         (standup-with-statenet)
         )
        (t ;; :lost-target
         (standup-with-statenet)
         (send *ri* :go-pos -0.7 0 0)
         (standup-with-statenet)
         (warn "~%;;; search target ;;;~%~%")
         (search-target)
         (standup-with-statenet)
         )
        ))
    ))

(warn "
(make-random-first-position)
(ball-kick-demo)
")
