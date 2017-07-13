(ros::roseus-add-msgs "geometry_msgs")

(setq *msg* nil)
(defun callback (msg)
  (setq *msg* msg))

(ros::roseus "test_offsensor")

(ros::subscribe "/off_rfsensor" geometry_msgs::WrenchStamped #'callback)

(ros::spin-once)
(setq *msg* nil)

(let ((st (ros::time-now)))
  (ros::rate 10)
  (while t
    (ros::sleep)
    (ros::spin-once)
    (when *msg*
      (exit 0))
    (when (> (send (ros::time- (ros::time-now) st) :to-sec) 10)
      (warn ";; subscribe timeout~%")
      (exit 1))
    ))
