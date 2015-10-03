(load "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-interface.l")
(jaxon_jvrc-init)
(setq *robot* *jaxon_jvrc*)

(send *ri* :set-gait-generator-param :optional-go-pos-finalize-footstep-num 0) ;; warn, for simulator only!
(defun start-walking ()
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  )

(defun speedup-mode ()
  (start-walking)
  (send *ri* :set-gait-generator-param :default-step-time 0.6)
  (send *ri* :set-gait-generator-param :stride-parameter (float-vector 0.25 0.1 15.0 0.1))
  (send *robot* :reset-pose)
  (send *robot* :legs :move-end-pos (float-vector 0 0 200) :local)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun default-mode ()
  (start-walking)
  (send *ri* :set-gait-generator-param :default-step-time 1.2)
  (send *ri* :set-gait-generator-param :stride-parameter (float-vector 0.15 0.05 10.0 0.05))
  (send *robot* :reset-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun kick-mode ()
  (start-walking)
  (walking-pose *robot* :root-link-pitch-offset 25)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun go-pos-time (x y r)
  (setq start-time (send (ros::time-now) :to-sec))
  (send *ri* :go-pos x y r)
  (format t "total time: ~A" (- (send (ros::time-now) :to-sec) start-time))
  )

(defun walking-pose
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (head-link-pitch-offset 0)
        (root-link-roll-offset 0)
        (chest-link-pitch-offset 0)
        (chest-link-roll-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose))
  "Generate and set walking pose.
   default-pose-method is initial pose, reset-manip-pose by default.
   Generated pose is near from default-pose-method pose.
   root-link-height-offset is root height offset [mm] from default-pose-method.
   root-link-pitch-offset and root-link-roll-offset are root pitch and roll offset [deg] from default-pose-method."
  (send robot default-pose-method)
  (send robot :fix-leg-to-coords fix-coords)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (send robot :move-coords
          (send
           (send
            (send (send (car (send robot :links)) :copy-worldcoords)
                  :translate (float-vector 0 0 root-link-height-offset))
            :rotate (deg2rad root-link-pitch-offset) :y)
           :rotate (deg2rad root-link-roll-offset) :x)
          (car (send robot :links)))
    (if (find-method robot :torso-waist-p) (send robot :torso-waist-p :joint-angle chest-link-pitch-offset))
    (if (find-method robot :torso-waist-r) (send robot :torso-waist-r :joint-angle chest-link-roll-offset))
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (if (find-method robot :head-neck-p) (send robot :head-neck-p :joint-angle head-link-pitch-offset))
    (send robot :angle-vector)
    ))

(defun main-loop ()
  (ros::advertise-service "/speedup_mode" std_srvs::Empty #'(lambda (req) (speedup-mode) (send req :response)))
  (ros::advertise-service "/default_mode" std_srvs::Empty #'(lambda (req) (default-mode) (send req :response)))
  (ros::rate 10)
  (while t
    (ros::sleep)
    (ros::spin-once)
    )
  )

(warning-message 2 "speedup-mode")
(warning-message 2 "default-mode")
(warning-message 2 "kick-mode")
