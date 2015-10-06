(require "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-interface.l")
(when (not (boundp '*jaxon_jvrc*))
  (jaxon_jvrc-init)
  (setq *robot* *jaxon_jvrc*)
  )
(send *ri* :set-st-param :cp-check-margin (* 60 1e-3))
(send *ri* :set-st-param :emergency-check-mode 2)
(unless (equal (send *ri* :get-auto-balancer-controller-mode) :mode_abc)
  (send *ri* :start-auto-balancer))
(unless (equal (send *ri* :get-st-controller-mode) :mode_st)
  (send *ri* :start-st))

(ros::roseus "push_recovery_test")

(defun pr-first-foot (fs)
  (cond ((< (aref fs 1) 0)
         :lleg)
        (t
         :rleg)))

(defun pr-second-foot (fs)
  (cond ((< (aref fs 1) 0)
         :rleg)
        (t
         :lleg)))

(defun remove-cp-offset ()
  (setq *step-time* 0.4)
  (setq *step-height* 90)
  (setq cp-offset (v- (send (send *ri* :get-st-param) :act_capture_point) (send (send *ri* :get-st-param) :ref_capture_point))))

(defun start-push-recovery-loop (&key (test-mode nil) (step-mode 0))
  (remove-cp-offset)
  (send *ri* :emergency-release-motion)
  (warning-message 2 "start push-recovery loop~%")
  (do-until-key
   (cond (test-mode
          (when (= (send *ri* :state :emergency-mode) 1)
            (cond ((= step-mode 1)
                   (push-recovery-single :test-mode t)) ;; not support
                  ((= step-mode 2)
                   (push-recovery-double :test-mode t)) ;; not support
                  (t
                   (push-recovery :test-mode t)))))
         (t
          (when (= (send *ri* :state :emergency-mode) 1)
            (cond ((= step-mode 1)
                   (push-recovery-single)) ;; not support
                  ((= step-mode 2)
                   (push-recovery-double)) ;; not support
                  (t
                   (push-recovery))))))))

(defun push-recovery (&key (test-mode nil))
  (let ((foot-step (scale 1e3 (v- (send (send *ri* :get-st-param) :act_capture_point) cp-offset))))
    (cond (test-mode
           (warn "foot-step : ~A~%" foot-step)
           (warn "fist-foot : ~A~%" (pr-first-foot foot-step)))
          (t
           (cond ((and (< (aref foot-step 0) 350) (< (aref foot-step 1) 70))
                  (send *ri* :set-foot-steps-with-param
                        (list (make-coords :coords (send *robot* (pr-second-foot foot-step) :end-coords :copy-worldcoords) :name (pr-second-foot foot-step))
                              (make-coords :coords (send (send *robot* (pr-first-foot foot-step) :end-coords :copy-worldcoords)
                                                         :translate (float-vector (aref foot-step 0) (aref foot-step 1) 0)) :name (pr-first-foot foot-step))
                              (make-coords :coords (send (send *robot* (pr-second-foot foot-step) :end-coords :copy-worldcoords)
                                                         :translate (float-vector (aref foot-step 0) (aref foot-step 1) 0)) :name (pr-second-foot foot-step)))
                        (list 0 *step-height* *step-height*)
                        (list 0.0 *step-time* *step-time*)
                        (list 0 0 0)
                        (list 0 0 0))
                 (warn "foot-step : ~A~%" foot-step)
                 (warn "fist-foot : ~A~%" (pr-first-foot foot-step)))
                 (t
                  (warning-message 1 "foot-step is too large : ~A~%" foot-step))))))
  (unix:sleep 1)
  (send *ri* :emergency-release-motion)
  (unix:sleep 1))
