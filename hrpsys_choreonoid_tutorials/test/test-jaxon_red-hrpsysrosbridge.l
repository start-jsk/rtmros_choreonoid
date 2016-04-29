#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(deftest test-init
  (assert (jaxon_red-init)))

(deftest test-angle-vector
  (assert (send *jaxon_red* :reset-pose))
  (assert (progn
            (send *ri* :angle-vector (send *jaxon_red* :angle-vector))
            (send *ri* :wait-interpolation))))

(run-all-tests)
(exit 0)
