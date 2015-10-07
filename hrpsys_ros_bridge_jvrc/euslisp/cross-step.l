(require "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-interface.l")
(require "package://drc_task_common/euslisp/test-drc-terrain-walk.l")
(when (not (boundp '*jaxon_jvrc*))
  (jaxon_jvrc-init)
  (setq *robot* *jaxon_jvrc*)
  )
(unless (equal (send *ri* :get-auto-balancer-controller-mode) :mode_abc)
  (send *ri* :start-auto-balancer))
(unless (equal (send *ri* :get-st-controller-mode) :mode_st)
  (send *ri* :start-st))

(send *ri* :set-gait-generator-param
      :optional-go-pos-finalize-footstep-num 0
      :default-step-time 0.8
      :stride-parameter (float-vector 0.25 0.1 15.0 0.1))
(send *ri* :set-auto-balancer-param :adjust-footstep-transition-time 0.5)
(walking-pose *robot*
              :default-pose-method :reset-pose
              :root-link-height-offset -110
              :root-link-pitch-offset 20
              :chest-link-pitch-offset 20)
(send *robot* :head :neck-p :joint-angle 35)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)

(setq *cross-step-stride* (list 250 100 10))
(setq *width-offset* 20)

(defun start-cross ()
  (send *ri* :set-gait-generator-param
        :default-orbit-type :cross
        :default-step-time 1.4
        :default-step-height 0.08)
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
              (make-coords :pos (float-vector (- (car *cross-step-stride*)) *width-offset* 0) :name :rleg)))
  )

(defun stop-cross ()
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
              (make-coords :pos (float-vector 0 -200 0) :name :rleg)))
  (send *ri* :set-gait-generator-param
        :default-orbit-type :cycloid
        :default-step-height 0.07
        :default-step-time 0.8)
  )

(defun cross-step (x y yaw)
  (let ((x-times (truncate (/ (* (abs x) 1000) (* 2 (car *cross-step-stride*)))))
        (x-mod (mod (truncate (* x 1000)) (* 2 (car *cross-step-stride*))))
        (y-times (truncate (/ (* (abs y) 1000) (cadr *cross-step-stride*))))
        (y-mod (mod (truncate (* y 1000)) (cadr *cross-step-stride*)))
        (yaw-times (truncate (/ (abs yaw) (caddr *cross-step-stride*))))
        (yaw-mod (mod yaw (caddr *cross-step-stride*)))
        (x-y)
        (y-x))
    ;; (warn "x-times~A~%" x-times)
    ;; (warn "y-times~A~%" y-times)
    ;; (warn "yaw-times~A~%" yaw-times)
    ;; (warn "x-mod~A~%" x-mod)
    ;; (warn "y-mod~A~%" y-mod)
    ;; (warn "yaw-mod~A~%" yaw-mod)
    (if (> x-times y-times)
        (setq x-y (- x-times y-times))
      (setq y-x (- y-times x-times)))
    (if x-y
        (if (> x 0)
            (progn
              (dotimes (i y-times)
                (when (y-or-n-p "1:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (car *cross-step-stride*))) (* 1e-3 (cadr *cross-step-stride*)) 0)
                  (send *ri* :set-foot-steps
                        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                              (make-coords :pos (float-vector (car *cross-step-stride*) (+ (cadr *cross-step-stride*) *width-offset*)  0) :name :rleg)
                              (make-coords :pos (float-vector (* 2 (car *cross-step-stride*)) (cadr *cross-step-stride*) 0) :name :lleg)))))
              (when (y-or-n-p "2:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (car *cross-step-stride*))) (* 1e-3 y-mod) 0)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (car *cross-step-stride*) (+ y-mod *width-offset* ) 0) :name :rleg)
                            (make-coords :pos (float-vector (* 2 (car *cross-step-stride*)) y-mod 0) :name :lleg))))
              (dotimes (i (- x-y 1))
                (when (y-or-n-p "3:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (car *cross-step-stride*))) 0 0)
                  (send *ri* :set-foot-steps
                        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                              (make-coords :pos (float-vector (car *cross-step-stride*) *width-offset* 0) :name :rleg)
                              (make-coords :pos (float-vector (* 2 (car *cross-step-stride*)) 0 0) :name :lleg)))))
              (when (not (= x-mod 0))
                (when (y-or-n-p "4:go-pos ~A ~A ~A~%" (* 1e-3 x-mod) 0 0)
                  (send *ri* :set-gait-generator-param
                        :default-orbit-type :stair
                        :default-step-time 0.8)
                  (send *ri* :set-foot-steps
                        (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                              (make-coords :pos (float-vector (+ (/ x-mod 3.0) (car *cross-step-stride*)) (- *width-offset*) 0) :name :lleg)
                              (make-coords :pos (float-vector (/ x-mod 3.0) 0 0) :name :rleg)
                              (make-coords :pos (float-vector (+ (* 2 (/ x-mod 3.0)) (car *cross-step-stride*)) (- *width-offset*) 0) :name :lleg)
                              (make-coords :pos (float-vector (* 2 (/ x-mod 3.0)) 0 0) :name :rleg)
                              (make-coords :pos (float-vector (+ (* 3 (/ x-mod 3.0)) (car *cross-step-stride*)) (- *width-offset*) 0) :name :lleg)
                              (make-coords :pos (float-vector (* 3 (/ x-mod 3.0)) 0 0) :name :rleg))))))
          (progn
            (dotimes (i y-times)
              (when (y-or-n-p "5:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (- (car *cross-step-stride*)))) (* 1e-3 (cadr *cross-step-stride*)) 0)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                            (make-coords :pos (float-vector (- (car *cross-step-stride*)) (- (cadr *cross-step-stride*) *width-offset* )  0) :name :lleg)
                            (make-coords :pos (float-vector (* 2 (- (car *cross-step-stride*))) (cadr *cross-step-stride*) 0) :name :rleg)))))
            (when (y-or-n-p "6:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (- (car *cross-step-stride*)))) (* 1e-3 y-mod) 0)
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                          (make-coords :pos (float-vector (- (car *cross-step-stride*)) (- y-mod *width-offset* ) 0) :name :lleg)
                          (make-coords :pos (float-vector (* 2 (- (car *cross-step-stride*))) y-mod 0) :name :rleg))))
            (dotimes (i (- x-y 1))
              (when (y-or-n-p "7:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (- (car *cross-step-stride*)))) 0 0)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                            (make-coords :pos (float-vector (- (car *cross-step-stride*)) (- *width-offset*) 0) :name :lleg)
                            (make-coords :pos (float-vector (* 2 (- (car *cross-step-stride*))) 0 0) :name :rleg)))))
            (when (not (= x-mod 0))
              (when (y-or-n-p "8:go-pos ~A ~A ~A~%" (* 1e-3 x-mod) 0 0)
                (send *ri* :set-gait-generator-param
                      :default-orbit-type :stair
                      :default-step-time 0.8)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (+ (/ x-mod 3.0) (- (car *cross-step-stride*))) *width-offset* 0) :name :rleg)
                            (make-coords :pos (float-vector (/ x-mod 3.0) 0 0) :name :lleg)
                            (make-coords :pos (float-vector (+ (* 2 (/ x-mod 3.0)) (- (car *cross-step-stride*))) *width-offset* 0) :name :rleg)
                            (make-coords :pos (float-vector (* 2 (/ x-mod 3.0)) 0 0) :name :lleg)
                            (make-coords :pos (float-vector (+ (* 3 (/ x-mod 3.0)) (- (car *cross-step-stride*))) *width-offset* 0) :name :rleg)
                            (make-coords :pos (float-vector (* 3 (/ x-mod 3.0)) 0 0) :name :lleg)))))))
      (if (> x 0)
          (progn
            (dotimes (i x-times)
              (when (y-or-n-p "9:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (car *cross-step-stride*))) (* 1e-3 (cadr *cross-step-stride*)) 0)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (car *cross-step-stride*) (+ (cadr *cross-step-stride*) *width-offset*)  0) :name :rleg)
                            (make-coords :pos (float-vector (* 2 (car *cross-step-stride*)) (cadr *cross-step-stride*) 0) :name :lleg)))))
            (send *ri* :set-gait-generator-param
                  :default-orbit-type :stair
                  :default-step-time 0.8)
            (dotimes (i (- y-x 1))
              (when (y-or-n-p "10:go-pos ~A ~A ~A~%" 0 (* 1e-3 (cadr *cross-step-stride*)) 0)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (- (car *cross-step-stride*)) (+ (cadr *cross-step-stride*) *width-offset*) 0) :name :rleg)
                            (make-coords :pos (float-vector 0 (cadr *cross-step-stride*) 0) :name :lleg)))))
            (when (or (not (= x-mod 0)) (not (= y-mod 0)))
              (when (y-or-n-p "11:go-pos ~A ~A ~A~%" (* 1e-3 x-mod) (* 1e-3 y-mod) 0)
                (send *ri* :set-gait-generator-param
                      :default-orbit-type :stair
                      :default-step-time 0.8)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                            (make-coords :pos (float-vector (+ (/ x-mod 3.0) (car *cross-step-stride*)) (- (/ y-mod 3.0) *width-offset*) 0) :name :lleg)
                            (make-coords :pos (float-vector (/ x-mod 3.0) (/ y-mod 3.0) 0) :name :rleg)
                            (make-coords :pos (float-vector (+ (* 2 (/ x-mod 3.0)) (car *cross-step-stride*)) (- (* 2 (/ y-mod 3.0)) *width-offset*) 0) :name :lleg)
                            (make-coords :pos (float-vector (* 2 (/ x-mod 3.0)) (* 2 (/ y-mod 3.0)) 0) :name :rleg)
                            (make-coords :pos (float-vector (+ (* 3 (/ x-mod 3.0)) (car *cross-step-stride*)) (- (* 3 (/ y-mod 3.0)) *width-offset*) 0) :name :lleg)
                            (make-coords :pos (float-vector (* 3 (/ x-mod 3.0)) (* 3 (/ y-mod 3.0)) 0) :name :rleg))))))
        (progn
          (dotimes (i x-times)
            (when (y-or-n-p "12:go-pos ~A ~A ~A~%" (* 1e-3 (* 2 (- (car *cross-step-stride*)))) (* 1e-3 (cadr *cross-step-stride*)) 0)
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                          (make-coords :pos (float-vector (- (car *cross-step-stride*)) (- (cadr *cross-step-stride*) *width-offset*)  0) :name :lleg)
                          (make-coords :pos (float-vector (* 2 (- (car *cross-step-stride*))) (cadr *cross-step-stride*) 0) :name :rleg)))))
          (send *ri* :set-gait-generator-param
                :default-orbit-type :stair
                :default-step-time 0.8)
          (dotimes (i y-x)
            (when (y-or-n-p "13:go-pos ~A ~A ~A~%" 0 (* 1e-3 (cadr *cross-step-stride*)) 0)
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :rleg)
                          (make-coords :pos (float-vector (car *cross-step-stride*) (- (cadr *cross-step-stride*) *width-offset*) 0) :name :lleg)
                          (make-coords :pos (float-vector 0 (cadr *cross-step-stride*) 0) :name :rleg)))))
          (when (or (not (= x-mod 0)) (not (= y-mod 0)))
            (when (y-or-n-p "14:go-pos ~A ~A ~A~%" (* 1e-3 x-mod) (* 1e-3 y-mod) 0)
              (send *ri* :set-gait-generator-param
                    :default-orbit-type :stair
                    :default-step-time 0.8)
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                          (make-coords :pos (float-vector (+ (/ x-mod 3.0) (- (car *cross-step-stride*))) (+ *width-offset* (/ y-mod 3.0)) 0) :name :rleg)
                          (make-coords :pos (float-vector (/ x-mod 3.0) (/ y-mod 3.0) 0) :name :lleg)
                          (make-coords :pos (float-vector (+ (* 2 (/ x-mod 3.0)) (- (car *cross-step-stride*))) (+ *width-offset* (* 2 (/ y-mod 3.0))) 0) :name :rleg)
                          (make-coords :pos (float-vector (* 2 (/ x-mod 3.0)) (* 2 (/ y-mod 3.0)) 0) :name :lleg)
                          (make-coords :pos (float-vector (+ (* 3 (/ x-mod 3.0)) (- (car *cross-step-stride*))) (+ *width-offset* (* 3 (/ y-mod 3.0))) 0) :name :rleg)
                          (make-coords :pos (float-vector (* 3 (/ x-mod 3.0)) (* 3 (/ y-mod 3.0)) 0) :name :lleg))))))))
    (unless (= yaw 0)
      (send *ri* :set-gait-generator-param
            :default-orbit-type :stair
            :default-step-time 0.8)
      (if (> yaw 0)
          (progn
            (dotimes (i yaw-times)
              (when (y-or-n-p "15:go-pos ~A ~A ~A~%" 0 0 (caddr *cross-step-stride*))
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (+ (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad (caddr *cross-step-stride*))))) (+ *width-offset* (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad (caddr *cross-step-stride*))))) 0) :rpy (list (deg2rad (caddr *cross-step-stride*)) 0 0) :name :rleg)
                            (make-coords :pos (float-vector (- (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad (caddr *cross-step-stride*))))) (- (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad (caddr *cross-step-stride*))))) 0) :rpy (list (deg2rad (caddr *cross-step-stride*)) 0 0) :name :lleg)))))
            (unless (= yaw-mod 0)
              (when (y-or-n-p "16:go-pos ~A ~A ~A~%" 0 0 yaw-mod)
                (send *ri* :set-foot-steps
                      (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                            (make-coords :pos (float-vector (+ (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad yaw-mod)))) (+ *width-offset* (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad yaw-mod)))) 0) :rpy (list (deg2rad yaw-mod) 0 0) :name :rleg)
                            (make-coords :pos (float-vector (- (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad yaw-mod)))) (- (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad yaw-mod)))) 0) :rpy (list (deg2rad yaw-mod) 0 0) :name :lleg))))))
        (progn
          (dotimes (i yaw-times)
            (when (y-or-n-p "17:go-pos ~A ~A ~A~%" 0 0 (- (caddr *cross-step-stride*)))
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                          (make-coords :pos (float-vector (+ (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (- (deg2rad (caddr *cross-step-stride*)))))) (+ *width-offset* (* (/ (- (car *cross-step-stride*)) 2.0) (sin (- (deg2rad (caddr *cross-step-stride*)))))) 0) :rpy (list (- (deg2rad (caddr *cross-step-stride*))) 0 0) :name :rleg)
                          (make-coords :pos (float-vector (- (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (- (deg2rad (caddr *cross-step-stride*)))))) (- (* (/ (- (car *cross-step-stride*)) 2.0) (sin (- (deg2rad (caddr *cross-step-stride*)))))) 0) :rpy (list (- (deg2rad (caddr *cross-step-stride*))) 0 0) :name :lleg)))))
          (unless (= yaw-mod 0)
            (when (y-or-n-p "18:go-pos ~A ~A ~A~%" 0 0 yaw-mod)
              (send *ri* :set-foot-steps
                    (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
                          (make-coords :pos (float-vector (+ (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad yaw-mod)))) (+ *width-offset* (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad yaw-mod)))) 0) :rpy (list (deg2rad yaw-mod) 0 0) :name :rleg)
                          (make-coords :pos (float-vector (- (/ (- (car *cross-step-stride*)) 2.0) (* (/ (- (car *cross-step-stride*)) 2.0) (cos (deg2rad yaw-mod)))) (- (* (/ (- (car *cross-step-stride*)) 2.0) (sin (deg2rad yaw-mod)))) 0) :rpy (list (deg2rad yaw-mod) 0 0) :name :lleg))))))))
    (send *ri* :set-gait-generator-param
          :default-orbit-type :cross
          :default-step-time 1.4)))
