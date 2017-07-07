(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")

(defun initialize-robot-for-walk-to-target ()
  (warn ";; create interface for robot~%")
  (unless (boundp '*ri*)
    (jaxon_jvrc-init))
  (unless (boundp '*robot*)
    (setq *robot* *jaxon_jvrc*))
  (warn ";; robot move to initial pose~%")
  (send *ri* :stop-impedance :arms)
  ;;
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
  ;;
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;;
  (warn ";; finish initalizing~%")
  )

(defun setup-walk-to-target ()
  (send *robot* :reset-manip-pose)
  (setq *target-point* nil)
  (defun target-callback (msg)
    (setq *target-point* msg))

  (ros::subscribe "/calc_target_point/output"
                  geometry_msgs::PointStamped
                  #'target-callback)

  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))

  (setq *last-target* nil)
  )

(defun search-target ()
  (ros::rate 8)

  (send *robot* :reset-pose)
  (send *robot* :head :neck-p :joint-angle 20)
  (send *robot* :torso :wasit-y :joint-angle 25)
  (send *robot* :head  :neck-y  :joint-angle 30)
  ;;(send *robot* :head  :neck-p  :joint-angle 15)
  (send *ri* :angle-vector (send *robot* :angle-vector)
        nil :default-controller 0 :scale 3.0 :min-time 0.2)
  (send *ri* :wait-interpolation)
  (warn ";; initialize pose for searching~%")

  (ros::spin-once)
  (setq *target-point* nil)
  (setq *last-target* nil)

  (send *robot* :reset-pose)
  (send *robot* :torso :waist-y :joint-angle -25)
  (send *robot* :head  :neck-y  :joint-angle -30)
  (send *ri* :angle-vector (send *robot* :angle-vector) 12000)

  (warn ";; start searching a target~%")
  (let ((st (ros::time-now)))
    (while (ros::ok)
      (ros::spin-once)

      (when (and
             *target-point*
             (send *tfl* :wait-for-transform
                   "BODY"
                   (send *target-point* :header :frame_id)
                   (send *target-point* :header :stamp) 2.0))
        (let ((cds
               (send *tfl* :lookup-transform
                     "BODY"
                     (send *target-point* :header :frame_id)
                     (send *target-point* :header :stamp)
                     ))
              (pos (ros::tf-point->pos (send *target-point* :point)))
              )

          ;; stop motion
          (let ((av (send *ri* :reference-vector)))
            (send *ri* :angle-vector av 50
                  :default-controller 0 :min-time 0.01)
            (send *ri* :wait-interpolation))

          (send cds :transform (send (send *robot* :body_lk) :worldcoords) :world)
          ;; cds is camera-frame(world)
          (setq pos (send cds :transform-vector pos))

          (return-from search-target pos)
          ))
      (when (> (send (ros::time- (ros::time-now) st) :to-sec) 14)
        ;;timeout
        (return-from search-target)
        )
      )
    ))

(defun walk-to-target (&key (stop-threshold 700))
  (warn ";; start walking to the target~%")
  (ros::rate 4)
  (while (ros::ok)
    (ros::spin-once)
    (when (and
           *target-point*
           (send *tfl* :wait-for-transform
                 "BODY"
                 (send *target-point* :header :frame_id)
                 (send *target-point* :header :stamp) 2.0))
      (let ((cds
             (send *tfl* :lookup-transform
                   "BODY"
                   (send *target-point* :header :frame_id)
                   (send *target-point* :header :stamp)
                   ))
            (pos (ros::tf-point->pos (send *target-point* :point)))
            2d-pos
            )
        (setq *target-point* nil)
        (setq *last-target* (ros::time-now))
        (send cds :transform (send (send *robot* :body_lk) :worldcoords) :world)
        ;; cds is camera-frame(world)
        (setq pos (send cds :transform-vector pos))
        ;; pos is point (world)
        (pprint (list 'pos pos))
        (setq 2d-pos (float-vector (elt pos 0) (elt pos 1) 0))
        ;;
        ;;(send *robot* :reset-manip-pose)
        ;;(send *robot* :fix-leg-to-coords (make-coords))
        ;;(send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
        (send *robot* :head :look-at pos)
        ;;
        (let ((remain-step (send *ri* :get-remaining-foot-step-sequence)))
          (cond
           ((< (norm 2d-pos) stop-threshold)
            (when remain-step
              (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
              (send *ri* :go-stop)
              (warn ";;; finish walk to target ;;;~%")
              (return-from walk-to-target :find-target)
              ))
           (t
            (let* ((ref (send *ri* :reference-vector))
                   (tm (* 20 (norm (v- ref (send *robot* :angle-vector)))))
                   (xvel (/ (elt pos 0) 12000.0))
                   (thvel (/ (rad2deg (atan2 (elt pos 1) (elt pos 0))) 4))
                   )
              (setq xvel
                    (if (> xvel 0.15) 0.15
                      (if (< xvel -0.15) -0.15 xvel)))
              (setq thvel
                    (if (> thvel 10.0) 10.0
                      (if (< thvel -10.0) -10.0 thvel)))
              (when (< tm 600) (setq tm 600))
              (send *ri* :angle-vector
                    (send *robot* :angle-vector)
                    tm :default-controller 0.0 :min-time 0.29)
              ;; (pprint (list xvel thvel tm))
              (send *ri* :go-velocity xvel 0 thvel)
              ))
           ))
        ))
    (when *last-target*
      (let* ((now (ros::time-now))
             (sec (send (ros::time- now *last-target*) :to-Sec)))
        (when (> sec 2.0) ;; lost target
          (setq *last-target* nil)
          (warn ";;; lost target ;;;~%")
          (send *ri* :go-stop)
          (return-from walk-to-target :lost-target)
          ))
      )
    (ros::sleep)
    )
  )

(defun make-random-first-position (&key (x-range 1600.0) (y-range 3200.0) (th-range 45.0))
  (setq *random-state* (make-random-state))
  (let ((cds
         (make-coords :pos (float-vector (- (+ 2000 (random x-range)))
                                         (- (random y-range) (* 0.5 y-range))
                                         995)
                      :rpy (list (deg2rad (- (random th-range) (* 0.5 th-range)))
                                 0 0))))
    (reset-simulation :coords cds)
    cds
    ))
