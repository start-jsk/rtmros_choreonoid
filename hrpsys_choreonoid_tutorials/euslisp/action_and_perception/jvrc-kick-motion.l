(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")

(defun setup-kick-motion (&key (initialize-robot-pose t)
                               (initial-pose :reset-pose))
  (unless (boundp '*ri*)
    (jaxon_jvrc-init))
  (unless (boundp '*robot*)
    (setq *robot* *jaxon_jvrc*))

  (when initialize-robot-pose
    (send *robot* initial-pose)
    (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
    (send *robot* :fix-leg-to-coords(make-coords) (list :rleg :lleg))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation))

  ;;
  ;; change parameters
  ;;
  (send *ri* :stop-impedance :arms)
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  ;;
  (send *ri* :set-auto-balancer-param
        :default-zmp-offsets (list #f(0 0 0) #f(0 0 0)
                                   #f(0 0 0) #f(0 0 0)))
  ;; no emergency stop
  (send *ri* :raw-set-st-param :emergency-check-mode 0);;
  ;; not working for EEFMCOP ???
  (send *ri* :set-st-param :st-algorithm "EEFM");;
  ;;
  (send *ri* :start-auto-balancer :limbs (list :rleg :lleg))
  (send *ri* :start-st)
  (send *ri* :go-pos 0 0 0) ;; dummy
  )

(defun change-to-single-leg-mode (swing-leg
                                  &key (leg-height 60) (base-time 500)
                                  (auto-balancer-transition 2.0))
  (if (= (length (send (send *ri* :get-auto-balancer-param) :leg_names)) 2)
      (progn
        ;; stop-auto-balancerして良いようにSequencePlayerにも指令値をおくる
        (send *robot* :fix-leg-to-coords
              (send *ri* :get-foot-step-param :dst-foot-midcoords) ;; require :go-po 0 0 0 ???
              '(:rleg :lleg))
        (send *robot* :angle-vector (send *ri* :state :reference-vector))
        (send *robot* :fix-leg-to-coords (make-coords))
        (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))

        (send *ri* :angle-vector-sequence-full
              (list
               (concatenate float-vector
                            (send *robot* :angle-vector)
                            (float-vector 0 0 0 0)) ;; this is for JAXON_JVRC
               )
              (list
               200
               )
              :root-coords
              (list
               (send (car (send *robot* :links)) :copy-worldcoords)
               )
              :zmp
              (list
               (send (send *robot* :foot-midcoords) :worldpos)
               )
              )
        (send *ri* :wait-interpolation-seq)

        ;; Stop -> 片足でstart
        (send *ri* :set-auto-balancer-param :transition-time 0.2)
        (send *ri* :stop-auto-balancer)
        ;;
        (send *ri* :set-auto-balancer-param
              :leg-names
              (list (case swing-leg (:rleg :lleg) (:lleg :rleg)))
              :transition-time auto-balancer-transition)
        (send *ri* :start-auto-balancer :limbs (list :rleg :lleg))
        ;;
        (send *ri* :set-auto-balancer-param :transition-time 1.4)

        ;; 片足あげる
        (let ((prev-av (send *robot* :angle-vector))
              rleg-av lleg-av)
          ;; set angle-vector just for legs
          (send *robot* :reset-pose)
          (setq rleg-av (send *robot* :rleg :angle-vector)
                lleg-av (send *robot* :lleg :angle-vector))
          (send *robot* :angle-vector prev-av)
          (send *robot* :rleg :angle-vector rleg-av)
          (send *robot* :lleg :angle-vector lleg-av)
          )
        (send *robot* :fix-leg-to-coords(make-coords) (list :rleg :lleg))
        (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
        (send *robot* swing-leg :move-end-pos (float-vector 0 0 leg-height) :world) ;;
        (send *ri* :angle-vector (send *robot* :angle-vector)
              base-time :default-controller 0 :min-time 0.1)
        (send *ri* :wait-interpolation)
        )
    (warn ";; Current mode is single leg mode~%")
    ))

(defun change-to-double-leg-mode ()
  (if (= (length (send (send *ri* :get-auto-balancer-param) :leg_names)) 1)
      (progn
        ;; 着地前姿勢
        (let ((prev-av (send *robot* :angle-vector))
              rleg-av lleg-av)
          ;; set angle-vector just for legs
          (send *robot* :reset-pose)
          (setq rleg-av (send *robot* :rleg :angle-vector)
                lleg-av (send *robot* :lleg :angle-vector))
          (send *robot* :angle-vector prev-av)
          (send *robot* :rleg :angle-vector rleg-av)
          (send *robot* :lleg :angle-vector lleg-av)
          )
        (send *robot* :fix-leg-to-coords (make-coords) (list :rleg :lleg))
        (send *robot*
              (case
                  (read-from-string
                   (format nil ":~A"
                           (car (send (send *ri* :get-auto-balancer-param) :leg_names))))
                (:rleg :lleg) (:lleg :rleg))
              :move-end-pos #f(0 0 100)
              :world)
        (send *ri* :angle-vector (send *robot* :angle-vector) 500
              :default-controller 0 :min-time 0.1)
        (send *ri* :wait-interpolation)

        ;; 着地
        (let ((prev-av (send *robot* :angle-vector))
              rleg-av lleg-av)
          ;; set angle-vector just for legs
          (send *robot* :reset-pose)
          (setq rleg-av (send *robot* :rleg :angle-vector)
                lleg-av (send *robot* :lleg :angle-vector))
          (send *robot* :angle-vector prev-av)
          (send *robot* :rleg :angle-vector rleg-av)
          (send *robot* :lleg :angle-vector lleg-av)
          )
        (send *robot* :fix-leg-to-coords(make-coords) (list :rleg :lleg))
        (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
        (send *ri* :angle-vector (send *robot* :angle-vector) 500
              :default-controller 0 :min-time 0.1)
        (send *ri* :wait-interpolation)

        ;; Stop -> 片足でstart
        (send *ri* :set-auto-balancer-param :transition-time 1.5)
        (send *ri* :stop-auto-balancer)
        ;;
        (send *ri* :set-auto-balancer-param :leg-names (list :rleg :lleg)
              :transition-time 0.2)
        (send *ri* :start-auto-balancer :limbs (list :rleg :lleg))
        (send *ri* :set-auto-balancer-param :transition-time 2.0)
        )
    (warn ";; Current mode is double leg mode~%")
    ))

(defun kick-motion (&key (swing-speed 700) (pre-crotch-p 15) (kick-crotch-p -65))
  (if (= (length (send (send *ri* :get-auto-balancer-param) :leg_names)) 1)
      (let ((swing-leg
             (case (read-from-string ;; check support leg of autobalancer
                    (format nil ":~A" (car (send (send *ri* :get-auto-balancer-param) :leg_names))))
               (:rleg :lleg)
               (:lleg :rleg))))

        (let (avs)
          ;; 後ろさげる
          (send *robot* swing-leg :crotch-p :joint-angle pre-crotch-p)
          (push (send *robot* :angle-vector) avs)
          ;; 蹴り出す
          (send *robot* swing-leg :crotch-p :joint-angle kick-crotch-p)
          (send *robot* swing-leg :ankle-p :joint-angle 10)
          (push (send *robot* :angle-vector) avs)

          (send *ri* :angle-vector-sequence
                (nreverse avs)
                (make-list (length avs) :initial-element swing-speed))
          (send *ri* :wait-interpolation)
          )
        )
    (warn ";; Cannot kick in double leg mode~%")
    ))

(defun reset-auto-balancer ()
  (send *ri* :set-auto-balancer-param :transition-time 0.7)
  (send *ri* :stop-auto-balancer)
        ;;
  (send *ri* :set-auto-balancer-param :leg-names (list :rleg :lleg)
        :transition-time 0.2)
  (send *ri* :start-auto-balancer :limbs (list :rleg :lleg))
  (send *ri* :set-auto-balancer-param :transition-time 1.4)
  )

(defun reset-kick-simulation (&rest args)
  ;; sometimes st does not stop by calling :stop-st
  (while (/= 0 (send (send *ri* :get-st-param) :controller_mode))
    ;;(print 'stop-st)
    (send *ri* :stop-st)
    (unix::usleep (* 200 1000)))
  ;;
  (apply #'reset-position args)
  ;;
  (reset-auto-balancer)
  (send *ri* :kalmanfilterservice_resetkalmanfilterstate)
  (unix::usleep (* 500 1000))
  (send *ri* :start-st)
  )
