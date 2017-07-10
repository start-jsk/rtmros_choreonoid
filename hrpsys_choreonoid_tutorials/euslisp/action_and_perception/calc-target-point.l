#!/usr/bin/env roseus

(ros::roseus-add-msgs "opencv_apps")
(ros::roseus-add-msgs "sensor_msgs")


(setq *left-msg* nil)
(setq *right-msg* nil)
(defun callback-left (msg)
  (setq *left-msg* msg))
(defun callback-right (msg)
  (setq *right-msg* msg))

(setq *left-camera-info* nil)
(setq *right-camera-info* nil)
(defun callback-left-camerainfo (msg)
  (setq *left-camera-info* msg))
(defun callback-right-camerainfo (msg)
  (setq *right-camera-info* msg))

(defun calc-target-point ()
  (ros::roseus "calc_target_point")

  (setq *timeout* 10.0) ;; sec
  (when (ros::has-param "~time_out")
    (setq *timeout* (ros::get-param "~time_out")))
  (ros::ros-info "timeout: ~A" *timeout*)

  (setq *use-input-kalman* t)
  (when (ros::has-param "~use_input_kalman")
    (setq *use-input-kalman* (ros::get-param "~use_input_kalman")))

  (setq *use-output-kalman* t)
  (when (ros::has-param "~use_output_kalman")
    (setq *use-output-kalman* (ros::get-param "~use_output_kalman")))

  (setq *area-threshold* 40)
  (when (ros::has-param "~area_threshold")
    (setq *area-threshold* (ros::get-param "~area_threshold")))

  (ros::advertise
   "~output"
   geometry_msgs::PointStamped)

  (ros::subscribe
   "left_camera_info"
   sensor_msgs::camerainfo
   #'callback-left-camerainfo)
  (ros::subscribe
   "right_camera_info"
   sensor_msgs::camerainfo
   #'callback-right-camerainfo)

  (ros::spin-once)
  (while (< (send (ros::time-now) :to-sec) 1.0)
    (unix::usleep (* 100 1000))
    (ros::spin-once))

  (let ((st (ros::time-now)))
    (while (not (and *left-camera-info*
                     *right-camera-info*))
      (when (> (send (ros::time- (ros::time-now) st) :to-sec)
               *timeout*)
        (warn ";; timeout, camera-info not found~%")
        (return-from calc-target-point nil))
      (unix::usleep (* 1000 100)) ;; sleep 100 ms
      (ros::spin-once)
      )
    )

  (ros::subscribe
   "left_moments"
   opencv_apps::MomentArrayStamped
   #'callback-left)

  (ros::subscribe
   "right_moments"
   opencv_apps::MomentArrayStamped
   #'callback-right)

  (ros::ros-info "found camera-info~%")

  (let (left-cam
        right-cam
        last-left-p
        last-right-p
        input-left-kf
        input-right-kf
        input-kf-initialized
        output-kk
        output-kf-initialized
        )
    (when *use-input-kalman*
      (require :kalmanlib "irteus/kalmanlib.l")
      (setq input-left-kf
            (instance kalman-filter :init :state-dim 4 :measure-dim 2
                      :q-variance 000.1 :r-variance (* 3.0 3.0)))
      (send input-left-kf :A #2f((1 0 1 0) (0 1 0 1) (0 0 1 0) (0 0 0 1)))
      (setq input-right-kf
            (instance kalman-filter :init :state-dim 4 :measure-dim 2
                      :q-variance 000.1 :r-variance (* 3.0 3.0)))
      (send input-right-kf :A #2f((1 0 1 0) (0 1 0 1) (0 0 1 0) (0 0 0 1)))
      )
    (when *use-output-kalman*
      (require :kalmanlib "irteus/kalmanlib.l")
      (setq output-kf
            (instance kalman-filter :init :state-dim 6 :measure-dim 3
                      :q-variance 000.1 :r-variance (* 50.0 50.0)))
      (send output-kf :A #2f((1 0 0 1 0 0)(0 1 0 0 1 0)(0 0 1 0 0 1)(0 0 0 1 0 0)(0 0 0 0 1 0)(0 0 0 0 0 1)))
      )
    (setq left-cam
          (make-camera-from-ros-camera-info-aux
           (send *left-camera-info* :width)
           (send *left-camera-info* :height)
           (send *left-camera-info* :P)
           nil))

    (setq right-cam
          (make-camera-from-ros-camera-info-aux
           (send *right-camera-info* :width)
           (send *right-camera-info* :height)
           (send *right-camera-info* :P) ;; fixed roseus
           ;;(let ((p (send *right-camera-info* :P)))
           ;;(setf (elt p 3) (* (/ (elt p 3) (elt p 0)) 1000.0))
           ;;p) ;; depend on roseus version
           nil))

    (send left-cam :assoc right-cam)

    (while (ros::ok)

      ;; get object position in image coordinates
      (setq *left-msg* nil)
      (setq *right-msg* nil)
      (ros::spin-once)

      (when (or *left-msg* *right-msg*)
        (when *left-msg*
          (let ((max-area 0)
                max-moment)
            (dolist (mom (send *left-msg* :moments))
              (when (and (not (c-isnan (send mom :center :x)))
                         (> (send mom :area) max-area))
                (setq max-area (send mom :area))
                (setq max-moment mom)))
            (when (and max-moment (> max-area *area-threshold*))
              (setq last-left-p (send max-moment :center))
              (send last-left-p :put :stamp (send *left-msg* :header :stamp)))
            (setq *left-msg* nil)
            ))
        (when *right-msg*
          (let ((max-area 0)
                max-moment)
            (dolist (mom (send *right-msg* :moments))
              (when (and (not (c-isnan (send mom :center :x)))
                         (> (send mom :area) max-area))
                (setq max-area (send mom :area))
                (setq max-moment mom)))
            (when (and max-moment (> max-area *area-threshold*))
              (setq last-right-p (send max-moment :center))
              (send last-right-p :put :stamp (send *right-msg* :header :stamp)))
            (setq *right-msg* nil)
            ))

        ;;calc mean_y
        (when (and last-left-p
                   last-right-p)
          ;; TODO: have to check synchronize left and right
          (ros::ros-info "")
          (ros::ros-info ";; msg received ~A ~A"
                            (send last-left-p :get :stamp)
                            (send last-right-p :get :stamp))

          (cond
           ((> (abs (- (send last-left-p  :y)
                       (send last-right-p :y)))
               20)
            )
           (t
            (let* ((mean-y (/ (+ (send last-left-p :y)
                                 (send last-right-p :y)) 2.0))
                   (left-x  (send last-left-p :x))
                   (right-x (send last-right-p :x))
                   left-line right-line
                   )
              (when *use-input-kalman*
                (cond
                 (input-kf-initialized
                  (let ((lp
                         (send input-left-kf  :proc  (float-vector (send last-left-p :x)   (send last-left-p :y))))
                        (rp
                         (send input-right-kf :proc (float-vector (send last-right-p :x)  (send last-right-p :y)))))
                    ;; over write filtered values
                    (setq left-x  (elt lp 0)
                          right-x (elt rp 0)
                          mean-y  (* 0.5 (+ (elt lp 1) (elt rp 1))))
                    (ros::ros-debug "~A ~A / ~A ~A"
                                    (send last-left-p :x) left-x
                                    (send last-right-p :x) right-x)
                    ))
                 (t
                  (send input-left-kf  :x (float-vector (send last-left-p :x)  (send last-left-p :y) 0 0))
                  (send input-right-kf :x (float-vector (send last-right-p :x) (send last-right-p :y) 0 0))
                  (setq input-kf-initialized t)
                  )))
              (ros::ros-info "x-l: ~A, x-r: ~A, y: ~A" left-x right-x mean-y)
              (when (> left-x right-x)
                (setq left-line  (make-line (send left-cam :worldpos)
                                            (v+ (send left-cam :worldpos)
                                                (scale 5000 (send left-cam :ray left-x mean-y)))))
                (setq right-line (make-line (send right-cam :worldpos)
                                            (v+ (send right-cam :worldpos)
                                                (scale 5000 (send right-cam :ray right-x mean-y)))))

                (let ((ln (send left-line :common-perpendicular right-line)))
                  ;; end-points of the line vertical both to left-line and right-line
                  (unless (eq ln :parallel)
                    (let* ((p (apply #'midpoint 0.5
                                     (send left-line :common-perpendicular right-line)))
                           (msg
                            (instance geometry_msgs::PointStamped :init
                                      :header (instance std_msgs::header :init
                                                        :frame_id (send *left-camera-info* :header :frame_id)
                                                        :stamp (send last-left-p :get :stamp))
                                      :point (ros::pos->tf-point p)))
                           )
                      (when *use-output-kalman*
                        (cond
                         (output-kf-initialized
                          (let ((prev-p p))
                            (setq p (send output-kf :proc prev-p))
                            (ros::ros-debug "prev-p/p ~A ~A" prev-p p))
                          (ros::ros-info "pub: ~A ~A ~A" (elt p 0) (elt p 1) (elt p 2))
                          (ros::publish "~output" msg)
                          )
                         (t
                          (send output-kf :x (float-vector (elt p 0) (elt p 1) (elt p 2) 0 0 0))
                          (setq output-kf-initialized t)
                          )))
                      )
                    ))
                );; /when
              (setq last-left-p nil
                    last-right-p nil)
              )
            ));; /cond
          );; /when calc-mean
        )
      )
    ))

(calc-target-point)
