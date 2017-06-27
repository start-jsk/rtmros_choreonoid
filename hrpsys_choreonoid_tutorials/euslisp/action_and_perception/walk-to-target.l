(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")

(require :choreonoid-service-client "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")

(require :walk-to-target "walk-to-target-lib.l")

(initialize-robot-for-walk-to-target)
(setup-walk-to-target)

(warn "
(make-random-first-position)
(walk-to-target)
")
