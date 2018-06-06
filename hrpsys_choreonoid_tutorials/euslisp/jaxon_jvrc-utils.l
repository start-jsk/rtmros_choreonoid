(require :jaxon_jvrc "package://hrpsys_choreonoid_tutorials/models/jaxon_jvrc.l")

(defmethod JAXON_JVRC-robot
  (:init-ending
   (&rest args)
   (prog1
       (send-super* :init-ending args)
     (when (member :define-min-max-table (send self :methods))
       (send self :define-min-max-table))
     (send self :add-additional-end-coords)
     (send self :add-thk-contact-coords);; should be end-coords
     (send self :add-hip-contact-coords);; should be end-coords
     ))
  (:add-additional-end-coords ()
   (dolist (ec-param (list
                      ;;(list :r_wrist_y_link_lk (make-coords :pos (float-vector 0 0 -185) :rpy (list 0 pi 0))
                      ;;:r-hand-tip-hard-point)
                      ;;(list :l_wrist_y_link_lk (make-coords :pos (float-vector 0 0 -185) :rpy (list 0 pi 0))
                      ;;:l-hand-tip-hard-point)
                      ;;(list :r_elbow_p_link_lk (make-coords :pos (float-vector -90 0 15) :rpy (list 0 -pi/2 0))
                      ;;:r-elbow-hard-point)
                      ;;(list :l_elbow_p_link_lk (make-coords :pos (float-vector -90 0 15) :rpy (list 0 -pi/2 0))
                      ;;:l-elbow-hard-point)
                      ;;(list :r_elbow_p_link_lk (make-coords :pos (float-vector -75 0 -265) :rpy (list 0 -pi/2 0))
                      ;;:r-wrist-hard-point)
                      ;;(list :l_elbow_p_link_lk (make-coords :pos (float-vector -75 0 -265) :rpy (list 0 -pi/2 0))
                      ;;:l-wrist-hard-point)
                      ;;(list :r_shoulder_r_link_lk (make-coords :pos (float-vector 130 -30 0) :rpy (list 0 pi/2 0))
                      ;;:r-shoulder-hard-point)
                      ;;(list :l_shoulder_r_link_lk (make-coords :pos (float-vector 130  30 0) :rpy (list 0 pi/2 0))
                      ;;:l-shoulder-hard-point)
                      ;;(list :r_shoulder_y_link_lk (make-coords :pos (float-vector -115 0 -90) :rpy (list 0 -pi/2 0))
                      ;;:r-upper-arm-hard-point)
                      ;;(list :l_shoulder_y_link_lk (make-coords :pos (float-vector -115 0 -90) :rpy (list 0 -pi/2 0))
                      ;;:l-upper-arm-hard-point)
                      ;;(list :r_crotch_p_link_lk (make-coords :pos (float-vector 120 0 -220) :rpy (list 0 pi/2 0))
                      ;;:r-thigh-hard-point)
                      ;;(list :l_crotch_p_link_lk (make-coords :pos (float-vector 120 0 -220) :rpy (list 0 pi/2 0))
                      ;;:l-thigh-hard-point)
                      (list :rleg_link3_lk (make-coords :pos (float-vector 80 -20 -60) :rpy (list 0 pi/2 0))
                            :r-knee-hard-point)
                      (list :lleg_link3_lk (make-coords :pos (float-vector 80  20 -60) :rpy (list 0 pi/2 0))
                            :l-knee-hard-point)
                      (list :rleg_link5_lk (make-coords :pos (float-vector 130 0 -100) :rpy (list 0 pi/2 0))
                            :r-foot-tip-hard-point)
                      (list :lleg_link5_lk (make-coords :pos (float-vector 130 0 -100) :rpy (list 0 pi/2 0))
                            :l-foot-tip-hard-point)
                      ;;(list :chest_p_link_lk  (make-coords :pos (float-vector 120 0 300) :rpy (list 0 pi/2 0))
                      ;;:body-front-hard-point)
                      ;;(list :chest_p_link_lk  (make-coords :pos (float-vector -140 0 300) :rpy (list 0 -pi/2 0))
                      ;;:body-back-hard-point)
                      ;;(list :body_lk (make-coords :pos (float-vector -80 0 -150) :rpy (list 0 -pi 0))
                      ;;:hip-hard-point)
                      ))
     (let ((ec (make-cascoords :name (caddr ec-param)
                               :coords (send (cadr ec-param) :transform
                                             (send self (car ec-param) :worldcoords)
                                             :world))))
       (send (send self (car ec-param)) :assoc ec)
       (push ec end-coords-list))
     )
   )

  (:add-thk-contact-coords
   (&key (offset (float-vector (- 15 12.765) 0 0)))
   (let* ((limb (list :rarm :larm))
          (name (list :rhand-contact-coords :lhand-contact-coords))
          tmpcec)
     (mapcar #'(lambda (l n sgn)
                 (setq tmpcec
                       (make-cascoords
                        :init :link-list
                        :parent (send self l :end-coords)
                        :coords (send (send (send (send self l :end-coords :copy-worldcoords)
                                                  :translate offset)
                                            :rotate -pi/2 :y)
                                      :rotate (* sgn -pi/2) :z)
                        :name n))
                 (send self :put n tmpcec)
                 (send (send self l :end-coords :parent) :assoc (send self :get n)))
             limb name (list +1 -1))))
  (:add-hip-contact-coords
   (&key (offset (float-vector -252 0 -175))) ; calculated from CAD data
   (let* ((name :hip-contact-coords)
          tmpcec)
     (setq tmpcec
           (make-cascoords
            :init :link-list :parent (car (send self :links))
            :coords
            (send
             (make-coords
              :pos (send (send (car (send self :links)) :copy-worldcoords) :worldpos)
              :rot (send (send (car (send self :links)) :copy-worldcoords) :worldrot))
             :translate offset :local)
            :name name))
     (send self :put name tmpcec)
     (send (car (send self :links)) :assoc (send self :get name))
     ))
  )
