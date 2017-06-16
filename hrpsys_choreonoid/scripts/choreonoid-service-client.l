(ros::roseus-add-srvs "roseus")

(unless (ros::ok)
  (ros::roseus "choreonoid_service_client"))

(defun reset-position (&key (robot "JAXON_RED")
                            (coords (make-coords :pos (float-vector 0 0 1000)))
                            (sleep 0.2) &allow-other-keys)
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
  ;;
  (when (/= (send *ri* :emergency-mode) 0)
    (send *ri* :emergency-release-motion)
    )
  ;;(send *ri* :stop-auto-balancer)
  (send *ri* :stop-st)
  ;;
  (apply #'reset-position args)
  (unix::usleep (* 200 1000)) ;; not needed ??
  (let ((strm (piped-fork "rosservice call /KalmanFilterServiceROSBridge/resetKalmanFilterState")))
    (close strm))
  ;;
  ;;(send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  )
