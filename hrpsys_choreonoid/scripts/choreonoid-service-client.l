(ros::roseus-add-srvs "roseus")
(ros::roseus-add-srvs "hrpsys_ros_bridge")

(unless (ros::ok)
  (ros::roseus "choreonoid_service_client"))

(defun reset-position (&key (robot "JAXON_RED")
                            (coords (make-coords :pos (float-vector 0 0 1000)))
                            (sleep 0.05) &allow-other-keys)
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

(defun pause-simulation (&key (robot "JAXON_RED"))
  (let ((req
         (instance roseus::StringStringRequest
                   :init :str
                   (format nil "callSimulation('~A','pauseSimulation')" robot))))
    (ros::service-call "/choreonoid_service" req)
    ))

(defun restart-simulation (&key (robot "JAXON_RED"))
  (let ((req
         (instance roseus::StringStringRequest
                   :init :str
                   (format nil "callSimulation('~A','restartSimulation')" robot))))
    (ros::service-call "/choreonoid_service" req)
    ))

(defun add-object-to-simulation (name filename &key (robot "JAXON_RED") (coords))
  ;; add as a static object
  (let ((reqstr
         (format nil "addObject('~A','~A'" name filename)))
    (when coords
      (let ((pos (send coords :worldpos))
            (rot (array-entity (send coords :worldrot))))
        (setq reqstr (format nil "~A,translation=[~A,~A,~A],rotation=[[~A,~A,~A],[~A,~A,~A],[~A,~A,~A]]"
                             reqstr
                             (/ (elt pos 0) 1000) (/ (elt pos 1) 1000) (/ (elt pos 2) 1000)
                             (elt rot 0) (elt rot 1) (elt rot 2)
                             (elt rot 3) (elt rot 4) (elt rot 5)
                             (elt rot 6) (elt rot 7) (elt rot 8)))
        ))
    (setq reqstr (format nil "~A)" reqstr))
    (ros::service-call "/choreonoid_service"
                       (instance roseus::StringStringRequest
                                 :init :str reqstr))
    ))

(defun get-coords-on-simulation (&key (robot "JAXON_RED") (link "WAIST"))
  (let ((req
         (instance roseus::StringStringRequest
                   :init :str
                   (format nil "getCoordinate('~A','~A')" robot link)))
        ret)
    (setq ret (ros::service-call "/choreonoid_service" req))
    (when ret
      (setq ret (send ret :str))
      (when (> (length ret) 0)
        (setq ret (read-from-string ret))
        (when (eq :success (car ret))
          (eval (cadr ret)))))
    ))

(defun reset-simulation (&rest args)
  (unless (boundp '*ri*)
    (warn "this function require *ri*~%")
    (return-from reset-simulation))

  ;; emergency
  (when (/= (send *ri* :emergency-mode) 0)
    (send *ri* :emergency-release-motion))

  ;; stop abc/st
  ;;(send *ri* :stop-auto-balancer) ;; stop-auto-balancer not required
  (send *ri* :stop-st)

  ;; set position on choreonoid
  (apply #'reset-position args)

  ;; 0.5 sec sleep with ros::time
  (let ((tm (ros::time+ (ros::time-now) (ros::time 0.2))))
    (while (ros::time< (ros::time-now) tm)
      (unix::usleep (* 100 1000))))
  ;; reset kalman filter
  (let ((req
         (instance hrpsys_ros_bridge::OpenHRP_KalmanFilterService_resetKalmanFilterStateRequest :init))
        ret)
    (setq ret (ros::service-call "/KalmanFilterServiceROSBridge/resetKalmanFilterState" req))
    (print-ros-msg ret)
    )
  ;; 1.0 sec sleep with ros::time
  (let ((tm (ros::time+ (ros::time-now) (ros::time 1.2))))
    (while (ros::time< (ros::time-now) tm)
      (unix::usleep (* 100 1000))))
  ;; restart abc/st
  ;;(send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  t
  )
