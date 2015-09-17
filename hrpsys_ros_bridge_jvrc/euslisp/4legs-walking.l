(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(defun setup-robot (&key (robot 'jaxon_red) (view t))
  ;; generate robot and object models
  (unless (boundp '*robot*)
    (progn
      (setq *robot* (funcall robot))
      (when view
        (objects (flatten
                  (list *robot*
                        (let (ret tmp (width 1500) (depth 1500) (height 1200))
                          (dotimes (i 3)
                            (dotimes (j 3)
                              (setq tmp (make-cube width depth height))
                              (send tmp :translate (float-vector (* i width) (* j depth) (/ height 2)))
                              (send tmp :set-color (float-vector (* 0.3 i) (- 1.0 (* 0.3 j)) (* 0.2 j)) 0.4)
                              (push tmp ret)))
                          ret)))))))
  (when view
    (send *irtviewer* :change-background #f(0.3 0.5 0.7))
    (send *irtviewer* :draw-objects))
  )

;; (warn "(reset-pose-to-touch-down)~%")
(defun reset-pose-to-touch-down
  (&key (offset (float-vector 0 0 0)))
  (let (ret)
    ;; reset pose
    (if (equal (send *robot* :name) "STARO")
        (send *robot* :torso :waist-p :max-angle 60))
    (send *irtviewer* :draw-objects)

    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (if (equal (send *robot* :name) "STARO")
        (send *robot* :move-centroid-on-foot :both '(:rarm :larm :rleg :lleg)
              :target-centroid-pos (float-vector 30 0 0))
      (send *robot* :move-centroid-on-foot :both '(:rarm :larm :rleg :lleg)))
    (push (list :angle-vector (send *robot* :angle-vector)
                :time 5000)
          ret)
    (send *irtviewer* :draw-objects)
    ;; contact
    (let ((tc
           (list
            (make-coords :pos (v+ (send *robot* :rleg :end-coords :worldpos)
                                  (float-vector 400 -250 0)))
            (make-coords :pos (v+ (send *robot* :lleg :end-coords :worldpos)
                                  (float-vector 400 +250 0)))
            (send *robot* :rleg :end-coords :copy-worldcoords)
            (send *robot* :lleg :end-coords :copy-worldcoords)))
          (ll
           (list (send *robot* :get :rhand-contact-coords)
                 (send *robot* :get :lhand-contact-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords)
                 )))
      ;;(print (mapcar #'(lambda (l) (send *robot* :link-list (send l :parent))) ll))
      (send *robot*
            :fullbody-inverse-kinematics tc
            :move-target ll
            :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send l :parent))) ll)
            :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                          (list (send *robot* :rarm :collar-y :child-link) 0.8)
                                          (list (send *robot* :larm :collar-y :child-link) 0.8)
                                          )
            :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
            :min (float-vector -100 -100 -1000 -90 -90 -90)
            :max (float-vector +100 +100 +1000 +90 +90 +90)
            :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.8 0.1 0.8 0.0)
            :debug-view :no-message)
      (send *robot* :fix-leg-to-coords (make-coords))
      (push (list :angle-vector (send *robot* :angle-vector)
                  :time 5000)
            ret))
    (send *irtviewer* :draw-objects)
    ;; touch down
    (let* ((tcl (mapcar #'(lambda (x) (send *robot* x :end-coords :copy-worldcoords))
                        (list :rarm :larm :rleg :lleg)))
           (tcp (scale (/ 1.0 4) (reduce #'v+ (send-all tcl :worldpos))))
           (target-z-pos 1000))
      (send *robot* :legs  :knee-p   :joint-angle 80)
      (send *robot* :legs  :crotch-p :joint-angle -80)
      (send *robot* :torso :waist-p  :joint-angle 30)
      (send *robot* :fix-leg-to-coords (make-coords))

      (with-move-target-link-list
       (mt ll *robot* '(:rarm :larm :rleg :lleg))
       (send *robot*
             :fullbody-inverse-kinematics tcl
             :move-target mt
             :link-list ll
             :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0))
             :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.5 0.1 0.5 0.0)
             :target-centroid-pos tcp))
      (push (list :angle-vector (send *robot* :angle-vector)
                  :time 5000)
            ret)
      (send *irtviewer* :draw-objects))
    (reverse ret)))

(warn "(init)~%")
(defun init ()
  (setup-robot :robot 'jaxon_red)
  (jaxon_red-init)
  (send *ri* :set-auto-balancer-param :transition-time 5)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-grasp))

(warn "(move-to-init-pose)~%")
(defun move-to-init-pose ()
  (dolist (avs (reset-pose-to-touch-down))
    (send *ri* :angle-vector (cadr (memq :angle-vector avs)))
    (send *ri* :wait-interpolation))
  (send *ri* :stop-auto-balancer))

(defun go-pos-quad (&key (x 0) (y 0) (th 0) (type :crawl) (debug-view nil))
  (let* ((gp (send *robot* :go-pos-params->footstep-list x y th
                   :forward-offset-length 100 :rotate-rad 10))
         (ik-args
          (list :min (float-vector -1e5 -1e5 -1e5 -180 -180 -180)
                :max (float-vector +1e5 +1e5 +1e5 +180 +180 +180)
                :absolute-p t
                :debug-view debug-view
                :cog-null-space nil
                :min-loop 2
                :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0))
                :root-link-virtual-joint-weight (float-vector 0.5 0.5 0.5 0.0 0.0 0.0)
                :cog-gain 5.0 :centroid-thre 50
                :init nil))
         (raec-from-rhcc (send (send *robot* :get :rhand-contact-coords)
                               :transformation (send *robot* :rarm :end-coords)))
         (laec-from-lhcc (send (send *robot* :get :lhand-contact-coords)
                               :transformation (send *robot* :larm :end-coords)))
         (first-leg (send (car gp) :name))
         (first-arm (case first-leg (:lleg :larm) (:rleg :rarm)))
         (second-leg (send (cadr gp) :name))
         (second-arm (case second-leg (:lleg :larm) (:rleg :rarm)))
         fsl)
    (dotimes (i (/ (length gp) 2))
      (let ((first-diff  (send (send *robot* first-leg :end-coords :copy-worldcoords)
                               :transformation (elt gp (+ (* i 2) 0)) :world))
            (second-diff (send (send *robot* second-leg :end-coords :copy-worldcoords)
                               :transformation (elt gp (+ (* i 2) 1)) :world)))
        (case type
          (:crawl
           (push (list (make-coords :coords (send (send *robot* first-arm :end-coords :copy-worldcoords)
                                                  :transform first-diff :world) :name first-arm)) fsl)
           (push (list (make-coords :coords (send (send *robot* first-leg :end-coords :copy-worldcoords)
                                                  :transform first-diff :world) :name first-leg)) fsl)
           (push (list (make-coords :coords (send (send *robot* second-arm :end-coords :copy-worldcoords)
                                                  :transform second-diff :world) :name second-arm)) fsl)
           (push (list (make-coords :coords (send (send *robot* second-leg :end-coords :copy-worldcoords)
                                                  :transform second-diff :world) :name second-leg)) fsl)
           )
          (:trot
           (push (list (make-coords :coords (send (send *robot* first-arm :end-coords :copy-worldcoords)
                                                  :transform first-diff :world) :name first-arm)
                       (make-coords :coords (send (send *robot* second-leg :end-coords :copy-worldcoords)
                                                  :transform first-diff :world) :name second-leg))
                 fsl)
           (push (list (make-coords :coords (send (send *robot* second-arm :end-coords :copy-worldcoords)
                                                  :transform second-diff :world) :name second-arm)
                       (make-coords :coords (send (send *robot* first-leg :end-coords :copy-worldcoords)
                                                  :transform second-diff :world) :name first-leg))
                 fsl))
          )))
    (if (= (mod (length gp) 2) 1)
        (let ((first-diff (send (elt gp 0) :transformation (car (last gp)) :world)))
          (case type
            (:crawl
             (push (list (make-coords :coords (send (send *robot* first-arm :end-coords :copy-worldcoords)
                                                    :transform first-diff :world) :name first-arm)) fsl)
             (push (list (make-coords :coords (send (send *robot* first-leg :end-coords :copy-worldcoords)
                                                    :transform first-diff :world) :name first-leg)) fsl))
            (:trot
             (push (list (make-coords :coords (send (send *robot* first-arm :end-coords :copy-worldcoords)
                                                    :transform first-diff :world) :name first-arm)
                         (make-coords :coords (send (send *robot* second-leg :end-coords :copy-worldcoords)
                                                    :transform first-diff :world) :name second-leg)
                         fsl))))
          ))
    ;; viewing
    (dolist (fs fsl)
      (dolist (f fs)
        (send f :draw-on :flush t :size 200)))

    (send *robot* :calc-walk-pattern-from-footstep-list
          (reverse fsl)
          :debug-view :no-message
          :all-limbs '(:rleg :lleg :rarm :larm)
          :init-pose-function #'(lambda () nil)
          :solve-angle-vector-args ik-args
          :default-step-height 20
          :default-step-time 0.8
          :dt 0.1
          )
    )
  )

(defun go-pos-quad-real (&key (x 0) (y 0) (th 0) ((:scale scl) 1))
  ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (let* ((fsn (go-pos-quad :x x :y y :th th :type :crawl))
         (tms (make-list (length fsn) :initial-element (* scl 100))))
    (send *ri* :angle-vector (cadr (memq :angle-vector (car fsn)))
          (* scl 100))
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector-sequence
          (mapcar #'(lambda (x) (cadr (memq :angle-vector x))) fsn) tms)
    (send *ri* :wait-interpolation)
    ))
