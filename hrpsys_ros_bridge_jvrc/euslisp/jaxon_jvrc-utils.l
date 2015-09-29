(require :jaxon_jvrc "package://hrpsys_ros_bridge_jvrc/models/jaxon_jvrc.l")

(defmethod JAXON_JVRC-robot
  (:init-ending
   (&rest args)
   (prog1
    (send-super* :init-ending args)
    (send self :add-thk-contact-coords)
    ))
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
  )
