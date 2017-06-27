(defun send-pose (&key (real nil) (base-time 5000) (wait nil) (wait-before t))
  (when real
    (when wait-before (send *ri* :wait-interpolation))
    (send *ri* :angle-vector (send *robot* :angle-vector) base-time)
    (when wait (send *ri* :wait-interpolation))
    ))

(defun mirror-angle-vector (&key (robot *robot*) (limbs (list :rleg :rleg)))
  (let ((zaxis (float-vector 0 0 1))
        (xaxis (float-vector 1 0 0)))
    (dolist (limb limbs)
      (let (ops-limb)
        (let ((str (string limb)))
          (cond
           ((= #\R (elt str 0))
            (setf (elt str 0) #\L)
            (setq ops-limb (intern str *keyword-package*))
            )
           ((= #\L (elt str 0))
            (setf (elt str 0) #\R)
            (setq ops-limb (intern str *keyword-package*))
            )
           (t
            (warn "mirror L->R or R->L (~A)~%" limb)
            )
           ))
        (when ops-limb
          (mapc
           #'(lambda (jj ops-jj)
               (let ((sign 1.0))
                 (when (or (eps-v= (jj . axis) zaxis)
                           (eps-v= (jj . axis) xaxis))
                   (setq sign -1.0))
                 (send ops-jj :joint-angle
                       (* sign (send jj :joint-angle)))
                 ))
           (send *robot* limb :joint-list)
           (send *robot* ops-limb :joint-list))
          )
        ))
    (send *robot* :angle-vector)
    ))

(defun fullbody-ik-jaxon (&key (stop 100)
                               (debug-view :no-message)
                               (centroid (send (send *robot* :foot-midcoords) :worldpos))
                               (centroid-thre 20.0);;
                               (larm-target (send *robot* :larm :end-coords :copy-worldcoords))
                               (rarm-target (send *robot* :rarm :end-coords :copy-worldcoords))
                               (lleg-target (send *robot* :lleg :end-coords :copy-worldcoords))
                               (rleg-target (send *robot* :rleg :end-coords :copy-worldcoords))
                               (translation-axis (list t t t t)) ;; rleg, lleg, rarm, larm
                               (rotation-axis    (list t t t t)) ;; rleg, lleg, rarm, larm
                               (thre  (list 0.2 0.2 0.2 0.2))    ;; rleg, lleg, rarm, larm
                               (rthre (list 0.017453 0.017453 0.017453 0.017453)) ;; rleg, lleg, rarm, larm
                               (root-link-virtual-joint-weight ;; root movement weight
                                (float-vector 0.1 0.1 0.1 0.05 0.2 0.2)) ;; x,y,z,roll,pitch,yaw
                               (min (float-vector -500 -500 -500 -20 -20 -10)) ;; root movement limit, x,y,z,roll,pitch,yaw
                               (max (float-vector  500  500  500  20  20  10)) ;; root movement limit, x,y,z,roll,pitch,yaw
                               (use-collision-avoidance nil)
                               (avoid-collision-null-gain 40)
                               (avoid-collision-joint-gain 0.1)
                               (collision-avoidance-pair
                                (list
                                 (list (send *robot* :larm_link2_lk)
                                       (send *robot* :chest_link2_lk))
                                 (list (send *robot* :larm_link2_lk)
                                       (send *robot* :chest_link1_lk))
                                 (list (send *robot* :rarm_link2_lk)
                                       (send *robot* :chest_link2_lk))
                                 (list (send *robot* :rarm_link2_lk)
                                       (send *robot* :chest_link1_lk))
                                 ;;(list (send *robot* :larm_link4_lk)
                                 ;;*shovel*)
                                 ))
                               (use-waist-r nil)
                               (use-waist-y nil)
                               (revert-if-fail t)
                               )
  (let* ((move-target-legs
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)))
         (move-target-arms
          (list (send *robot* :rarm :end-coords)
                (send *robot* :larm :end-coords)))
         (move-target (append move-target-legs move-target-arms))
         (target-coords
          (list
           rleg-target lleg-target
           rarm-target larm-target
           ))
         (additional-weight-list ;; jaxon specific
          (list
           (if (not use-waist-r)
               (list (send *robot* :torso :waist-r :child-link) 0)
             (if (numberp use-waist-r)
                 (list (send *robot* :torso :waist-r :child-link) use-waist-r)))
           (if (not use-waist-y)
               (list (send *robot* :torso :waist-y :child-link) 0)
             (if (numberp use-waist-y)
                 (list (send *robot* :torso :waist-y :child-link) use-waist-y)))
           ))
         (link-list)
         )
    (setq additional-weight-list (remove nil additional-weight-list));;

    ;;link-list
    (setq link-list
          (mapcar #'(lambda (limb)
                      (send *robot* :link-list (send limb :parent))) move-target))

    (let (args)
      (when use-collision-avoidance
        (push :avoid-collision-null-gain args)
        (push avoid-collision-null-gain args)
        (push :avoid-collision-joint-gain args)
        (push avoid-collision-joint-gain args)
        (push :collision-avoidance-link-pair args)
        (push collision-avoidance-pair args)
        (setq args (nreverse args))
        )
      (send* *robot* :fullbody-inverse-kinematics target-coords
             :move-target move-target
             :link-list link-list
             :translation-axis translation-axis
             :rotation-axis    rotation-axis
             :target-centroid-pos centroid
             :centroid-thre centroid-thre
             :thre thre
             :rthre rthre
             :min min
             :max max
             :additional-weight-list additional-weight-list
             :root-link-virtual-joint-weight root-link-virtual-joint-weight
             :debug-view debug-view ;;:no-message
             :stop stop ;;
             :revert-if-fail revert-if-fail
             :dump-command nil
             :warnp t
             args
             )
      )
    ))

(defun move-base-pose (coords &key (wrt :local) (waist-p)
                              (fullbody t)
                              (crotch-relative)
                              (centroid (send *robot* :centroid))
                              (fullbody-func #'fullbody-ik-jaxon)
                              (fullbody-args)
                              (rotation-axis (list t t t t)))
  (let ((rh (send *robot* :rarm :end-coords :copy-worldcoords))
        (lh (send *robot* :larm :end-coords :copy-worldcoords))
        (rl (send *robot* :rleg :end-coords :copy-worldcoords))
        (ll (send *robot* :lleg :end-coords :copy-worldcoords))
        (org-av (send *robot* :angle-vector))
        (org-coords (send *robot* :copy-worldcoords))
        (ret t)
        )

    (when waist-p
      (send *robot* :torso :waist-p :joint-angle waist-p))
    (when crotch-relative
      (send *robot* :legs :crotch-p :joint-angle crotch-relative :relative t)
      (send *robot* :move-coords rl (send *robot* :rleg :end-coords :worldcoords))
      )

    (send *robot* :move-coords
          (send (send *robot* :body_lk :copy-worldcoords) :transform coords wrt)
          (send *robot* :body_lk :worldcoords))

    (cond
     (fullbody
      (setq ret
            (apply fullbody-func
             :centroid centroid
             :larm-target lh
             :rarm-target rh
             :rleg-target rl
             :lleg-target ll
             :rotation-axis rotation-axis
             fullbody-args
             )))
     (t
      (when ret
        (setq ret (send *robot* :rleg :inverse-kinematics rl
                        :rotation-axis (elt rotation-axis 0)
                        )))
      (when ret
        (setq ret (send *robot* :lleg :inverse-kinematics ll
                        :rotation-axis (elt rotation-axis 1)
                        )))
      (when ret
        (setq ret (send *robot* :rarm :inverse-kinematics rh
                        :rotation-axis (elt rotation-axis 2)
                        )))
      (when ret
        (setq ret (send *robot* :larm :inverse-kinematics lh
                        :rotation-axis (elt rotation-axis 3)
                        )))
      ))

    (unless ret
      (send *robot* :angle-vector org-av)
      (send *robot* :newcoords org-coords))
    ret
    ))

(defun move-limbs (&key (height 50) (forward 200) (move-cent 50)
                        (side 0) (wrt :world)
                        (llimb :larm) (rlimb :rarm)
                        (real) (base-time 1000))
  (let (ret)
    (setq ret (fullbody-ik-jaxon :centroid (v+ (send *robot* :centroid) (float-vector 0 move-cent 0))))
    (unless ret
      (warn "~%~%~%;;;;; error 0~%~%~%"))
    (send-pose :real real :base-time base-time)
    (let ((avs) av
          (org-av (send *robot* :angle-vector))
          (tgt (send *robot* rlimb :end-coords :copy-worldcoords)))
      (setq av (send *robot* rlimb :move-end-pos (float-vector 0 0 height)     wrt))
      (unless av
        (setq av (send *robot* rlimb :move-end-pos (float-vector 0 0 height)     wrt :rotation-axis nil)))
      (push av avs)
      (setq av (send *robot* rlimb :move-end-pos (float-vector forward side 0) wrt))
      (unless av
        (setq av (send *robot* rlimb :move-end-pos (float-vector forward side 0) wrt :rotation-axis nil)))
      (push av avs)
      (send tgt :translate (float-vector forward side 0) wrt)
      (setq av (send *robot* rlimb :inverse-kinematics tgt))
      (push av avs)
      (unless (every #'identity avs)
        (warn "~%~%~%;;;;; error avs0~%~%~%"))
      (when real
        (send *ri* :wait-interpolation)
        (send *ri* :angle-vector-sequence
              (reverse avs) (make-list (length avs) :initial-element (/ base-time 3)))
        )
      )
    (setq ret (fullbody-ik-jaxon :centroid (v- (send *robot* :centroid) (float-vector 0 (* 2 move-cent) 0))))
    (unless ret
      (warn "~%~%~%;;;;; error 1~%~%~%"))
    (send-pose :real real :base-time base-time)

    (let ((avs) av
          (org-av (send *robot* :angle-vector))
          (tgt (send *robot* llimb :end-coords :copy-worldcoords)))
      (setq av (send *robot* llimb :move-end-pos (float-vector 0 0 height)     wrt))
      (unless av
        (setq av (send *robot* llimb :move-end-pos (float-vector 0 0 height)     wrt :rotation-axis nil)))
      (push av avs)
      (setq av (send *robot* llimb :move-end-pos (float-vector forward (- side) 0) wrt))
      (unless av
        (setq av (send *robot* llimb :move-end-pos (float-vector forward (- side) 0) wrt :rotation-axis nil)))
      (push av avs)
      (send tgt :translate (float-vector forward (- side) 0) wrt)
      (setq av (send *robot* llimb :inverse-kinematics tgt))
      (push av avs)
      (unless (every #'identity avs)
        (warn "~%~%~%;;;;; error avs1~%~%~%"))
      (when real
        (send *ri* :wait-interpolation)
        (send *ri* :angle-vector-sequence
              (reverse avs) (make-list (length avs) :initial-element (/ base-time 3)))
        )
      )
    (let ((cent (copy-seq (send *robot* :centroid))))
      (setf (elt cent 1) 0)
      (setq ret (fullbody-ik-jaxon :centroid cent))
      (unless ret
        (warn "~%~%~%;;;;; error 2~%~%~%"))
      (send-pose :real real :base-time base-time)
      )
    ))

(defun check-centroid-on-support-polygon
  (&key (robot *robot*)
        (poly0 :rleg-end-coords)
        (poly1 :lleg-end-coords)
        (poly2 :larm-end-coords)
        (draw t)
        )

  (let ((p0
         (send (send robot :end-coords poly0) :worldpos))
        (p1
         (send (send robot :end-coords poly1) :worldpos))
        (p2
         (send (send robot :end-coords poly2) :worldpos))
        (cent (send robot :centroid))
        pl ret
        )

    (setq pl ;; plane on p0
          (make-plane :normal (normalize-vector (v* (v- p1 p0) (float-vector 0 0 1)))
                      :point  (copy-seq p0)
                      ))

    (when draw
      (send (make-line p0 p1)
            :draw-on :flush t :color #f(0 1 0) :width 3)
      (send (send robot :centroid) :draw-on :flush t
            :size 250 :width 3 :color #f(1 0 0))
      )

    (let ((d2 (send pl :distance p2))
          (d1 (send pl :distance cent)))
      (cond
       ((or (and (> d2 0) (< d1 0))
            (and (< d2 0) (> d1 0)))
        ;; (warn "centroid OK~%");;
        )
       (t
        ;;(if (> d1 0)
        (setq ret (v+ cent (scale (- d1) (send pl :normal))));; desired cent
        ))
      )
    ret
    ))

(defun fix-end-coords-on-ground (ec0 ec1 ec2 &key (robot *robot*))
  ""
  (let* ((p0 (send (send robot :end-coords ec0) :worldpos))
         (p1 (send (send robot :end-coords ec1) :worldpos))
         (p2 (send (send robot :end-coords ec2) :worldpos))
         (v1 (v- p1 p0))
         (v2 (v- p2 p0))
         (nv (normalize-vector (v* (v- p1 p0) (v- p2 p0))))
         (z  (float-vector 0 0 1))
         (rot-v (if (> (v. nv z) 0) (v* nv z) (v* z nv)))
         )
    ;;(pprint (list (v. nv z) nv rot-v))
    (when (> (norm rot-v) 1e-5)
      ;; rotate
      (send robot :rotate (asin (norm rot-v)) (normalize-vector rot-v) :world)
      (send robot :worldcoords)
      )
    (let ((md (midpoint 0.5
                        (send (send robot :end-coords ec0) :worldpos)
                        (send (send robot :end-coords ec1) :worldpos))))
      ;; (pprint md)
      (send robot :translate (float-vector 0 0 (- (elt md 2))) :world)
      )
    robot
    ))

(defmacro revert-model-check (forms &key (robot *robot*))
  `(let ((org-av (send ,robot :angle-vector))
         (org-cds (send ,robot :copy-worldcoords)))
    (unwind-protect
        (progn
          (let ((robot ,robot))
            ,forms
            ))
      (progn
        (send ,robot :angle-vector org-av)
        (send ,robot :newcoords org-cds))
      ))
  )
