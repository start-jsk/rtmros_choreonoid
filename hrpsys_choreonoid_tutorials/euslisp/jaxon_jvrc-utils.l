(require :jaxon_jvrc "package://hrpsys_choreonoid_tutorials/models/jaxon_jvrc.l")

(defmethod JAXON_JVRC-robot
  (:init-ending
   (&rest args)
   (prog1
       (send-super* :init-ending args)
     (when (member :define-min-max-table (send self :methods))
       (send self :define-min-max-table))
     (send self :add-thk-contact-coords)
     (send self :add-hip-contact-coords)
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
