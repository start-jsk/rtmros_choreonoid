(ros::roseus-add-srvs "roseus")

(unless (ros::ok)
  (ros::roseus "choreonoid_service_client"))

(defun reset-position (&key (robot "JAXON_RED")
                            (coords (make-coords :pos (float-vector 0 0 1000)))
                            (sleep 0.2))
  (let ((pos (send coords :worldpos))
        (rpy (car (send coords :rpy-angle)))
        req)
    (setq pos (scale 0.001 pos))
    (setq req
          (instance roseus::StringStringRequest :init
                    :str (format nil "resetPosition('~A', [~A, ~A, ~A], [~A, ~A, ~A], ~A)"
                                 robot (elt pos 0) (elt pos 1) (elt pos 2)
                                 (elt rpy 0) (elt rpy 1) (elt rpy 2)
                                 sleep)))
    (ros::service-call "/choreonoid_service" req)
    ))

(defun add-external-force (&key (robot "JAXON_RED")
                                (link "WAIST")
                                (pos (float-vector 0 0 100)) ;; mm
                                (force (float-vector 100 0 0)) ;; N
                                (tm 1.0)) ;; sec
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

