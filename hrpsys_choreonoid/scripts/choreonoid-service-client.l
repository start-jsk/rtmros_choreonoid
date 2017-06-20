(ros::roseus-add-srvs "roseus")

(unless (ros::ok)
  (ros::roseus "choreonoid_service_client"))

(defun reset-position (&key (robot "JAXON_RED")
                            (coords (make-coords :pos (float-vector 0 0 1000)))
                            (sleep 0.1) &allow-other-keys)
  (let ((pos (send coords :worldpos))
        (rpy (car (send coords :rpy-angle)))
        req)
    (setq pos (scale 0.001 pos))
    (setq req
          (instance roseus::StringStringRequest :init
                    :str (format nil "resetPosition('~A', [~A, ~A, ~A], [~A, ~A, ~A], ~A)"
                                 robot (elt pos 0) (elt pos 1) (elt pos 2)
                                 (elt rpy 2) (elt rpy 1) (elt rpy 0)
                                 sleep)))
    (ros::service-call "/choreonoid_service" req)
    ))

(defun add-external-force (&key (robot "JAXON_RED")
                                (link "WAIST")
                                (pos (float-vector 0 0 100)) ;; mm
                                (force (float-vector 100 0 0)) ;; N
                                (tm 1.0) ;; sec
                                &allow-other-keys)
  (let (req)
    (setq pos (scale 0.001 pos))
    (setq req
          (instance roseus::StringStringRequest :init
                    :str (format nil "addExternalForce('~A', '~A', [~A, ~A, ~A], [~A, ~A, ~A], ~A)"
                                 robot link
                                 (elt pos 0) (elt pos 1) (elt pos 2)
                                 (elt force 0) (elt force 1) (elt force 2)
                                 tm)))
    (ros::service-call "/choreonoid_service" req)
    ))

(defun reset-simulation (&rest args)
  (unless (boundp '*ri*)
    (warn "this function require *ri*~%")
    (return-from reset-simulation))

  ;; emergency
  (when (/= (send *ri* :emergency-mode) 0)
    (send *ri* :emergency-release-motion)
    )

  ;; stop abc/st
  ;;(send *ri* :stop-auto-balancer) ;; stop-auto-balancer not required
  (send *ri* :stop-st)

  ;; set position on choreonoid
  (apply #'reset-position args)

  ;; reset kalmanfilter
  ;;(let ((strm (piped-fork "rosservice call /KalmanFilterServiceROSBridge/resetKalmanFilterState")))
  ;;(close strm))
  (unix::usleep (* 500 1000)) ;; not needed ??
  (let ((req
         (instance hrpsys_ros_bridge::OpenHRP_KalmanFilterService_resetKalmanFilterStateRequest :init))
        ret)
    (setq ret (ros::service-call "/KalmanFilterServiceROSBridge/resetKalmanFilterState" req))
    (print-ros-msg ret)
    )

  ;; restart abc/st
  ;;(send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  t
  )
