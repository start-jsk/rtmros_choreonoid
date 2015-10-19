(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(ros::roseus "test")
(require "package://drc_task_common/euslisp/primitive-marker-util.l")
(require "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-interface.l")
(require "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")
(ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)

(defun robot-init
  ()
  (jaxon_jvrc-init)
  ;;(send *ri* :start-auto-balancer)
  ;;(send *ri* :start-st)
  ;;(send *ri* :start-impedance :arms :d-p 600 :d-r 200)
  (setq *hand* (instance jvrc-hand-controller :init))
  (send *hand* :move-gripper :rarm 0)
  (send *hand* :move-gripper :larm 0)
;;  (send *ri* :start-impedance :arms)
  (send *ri* :wait-interpolation)
  (send *ri* :set-ref-force (float-vector 0 0 (* -3.0 9.8)) 2000 :rarm)
  (send *ri* :set-ref-force (float-vector 0 0 (* -3.0 9.8)) 2000 :larm)
  (setq *tf* *tfl*)
  )

(defun insert-debris3-marker
  ()
  (insert-primitive-marker :shape-type 3 :name "debris3" :resource "package://jvrc_models/models/debris3-test.dae")
  (ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
  (set-primitive-marker-color :r 0 :b 1 :g 1 :a 1)
  )

(defun insert-debris5-marker
  ()
  (insert-primitive-marker :shape-type 3 :name "debris5" :resource "package://jvrc_models/models/debris5-visual.dae")
  (ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
  (set-primitive-marker-color :r 0 :b 1 :g 1 :a 1)
  )

(defun remove-debris3
  ()
  (warning-message 3 "pose1")
  (read-line)
  (send *ri* :angle-vector #f(-0.391708 1.0984 -23.0047 40.9163 -21.8792 -0.881322 -0.385053 1.09814 -23.2598 41.17 -21.8919 -0.886796 -3.47934 2.47294 6.07775 -32.998 40.0 39.7952 69.3759 -54.0102 -21.9768 -90.1597 -17.4169 13.8688 15.9155 -0.009159 54.9755 19.9867 14.983 -99.9699 24.9999 -0.000757 -44.9915) 3000)
  (send *ri* :wait-interpolation)

  (warning-message 3 "pose2")
  (read-line)
  (send *ri* :angle-vector #f(-0.730436 -0.197759 -48.4931 85.6674 -43.0614 -0.183102 -0.723213 -0.198323 -48.5744 85.3224 -42.6401 -0.180739 -0.256492 -0.415069 8.04188 -32.9996 40.0 59.3714 32.1701 -45.2955 5.23996 -36.1627 -18.6377 28.9387 1.4425 -1.00659 45.6695 23.4603 3.98898 -83.2238 28.2654 3.8817 -56.9825) 3000)
  (send *ri* :wait-interpolation)

  (warning-message 3 "pose3")
  (read-line)
  (send *hand* :move-gripper :rarm 0.4)
  (send *ri* :wait-interpolation)
  (send *ri* :set-ref-force (float-vector 0 0 (* -3.0 9.8)) 2000 :rarm)
  (send *ri* :wait-interpolation)

  (warning-message 3 "go-pos")
  (read-line)
  (send *ri* :go-pos 0 0.2 0)

  (warning-message 3 "pose5")
  (read-line)
  (send *ri* :angle-vector #f(-0.730436 -0.197759 -48.4931 85.6674 -43.0614 -0.183102 -0.723213 -0.198323 -48.5744 85.3224 -42.6401 -0.180739 -0.256492 -0.415069 8.04188 -32.9996 40.0 59.3714 32.1701 -45.2955 5.23996 -36.1627 -18.6377 28.9387 1.4425 -1.00659 45.6695 23.4603 3.98898 -83.2238 28.2654 3.8817 -56.9825) 3000)
  (send *ri* :wait-interpolation)

  (warning-message 3 "pose6")
  (read-line)
  (send *ri* :angle-vector #f(-0.429246 -3.78377 -27.9545 41.8302 -20.7542 3.92987 -0.309221 -3.69319 -28.2789 42.0201 -19.1485 2.75158 5.32698 7.17634 6.9639 -33.0 40.0 47.9513 0.735376 -52.7258 30.1852 0.299052 14.7511 39.2139 -57.5538 -1.11739 44.0515 23.3686 1.56269 -83.3773 28.6018 6.42036 -56.5414) 3000)
  (send *ri* :wait-interpolation)

  (warning-message 3 "pose7")
  (read-line)
  (send *ri* :angle-vector #f(-0.413955 3.67549 -25.8525 40.8648 -19.7169 -2.67568 -0.400664 3.72953 -26.4881 41.7293 -19.9871 -3.18139 -1.00961 4.91703 13.1556 -7.73066 40.0 62.8994 33.2953 -44.5476 -3.59963 -61.0063 -36.8563 26.403 -0.451226 -1.16773 45.245 23.5009 4.01651 -83.3835 28.3355 4.04754 -56.3943) 3000)
  (send *ri* :wait-interpolation)

  (warning-message 3 "pose8")
  (read-line)
  (send *ri* :angle-vector #f(-0.39924 1.95589 -25.4272 40.7542 -19.649 -1.3167 -0.392909 1.95964 -26.0552 41.6809 -19.9329 -1.31646 -1.95276 2.08222 10.4144 -10.9277 40.0 57.1703 55.7116 -55.1346 -15.9284 -86.3631 -48.0269 27.3547 -2.9623 -1.07198 45.3986 23.486 4.23864 -82.9572 28.2367 3.56674 -56.8438) 3000)
  )

(defun remove-debris5
  ()
  (warning-message 3 "pose1")
  (read-line)
  (send *ri* :angle-vector #f(1.47043 2.11326 -51.8732 84.6015 -37.4325 -2.69681 1.45838 2.12309 -51.257 84.2121 -37.3241 -2.80411 -3.71543 10.0689 -4.04431 30.9985 40.0 1.27682 44.8749 -23.4951 -3.30065 -83.9951 -28.4826 -5.12628 -55.828 -45.2677 -3.02621 38.9694 21.7362 -14.3525 23.8971 5.22156 -48.825) 3000)

  (warning-message 3 "pose2")
  (read-line)
  (send *hand* :move-gripper :larm 0.2)

  ;;(send *ri* :set-ref-force (float-vector 0 0 (* -3.0 9.8)) 2000 :rarm)

  (warning-message 3 "pose3")
  (read-line)
  (send *ri* :angle-vector #f(4.92585 -0.427514 -48.7467 86.4398 -41.8737 -0.525173 4.96259 -0.326924 -46.5539 85.778 -43.7737 -0.917311 2.41178 -1.34775 -7.31147 30.9986 40.0 0.981345 45.7182 -23.4898 -4.88985 -82.4636 -28.0904 -2.62707 -57.2898 -59.731 -5.314 18.502 7.26523 -26.9456 6.9739 73.6127 -34.9736) 3000)

  (warning-message 3 "pose4")
  (read-line)
  (send *ri* :angle-vector #f(5.05837 0.110874 -42.8731 85.7622 -43.6836 -0.671854 5.08844 0.216938 -40.5494 85.0148 -45.9661 -1.05799 5.14223 -0.160349 -6.31351 24.1328 40.0 0.997376 45.7712 -23.5096 -5.20005 -82.3765 -28.0474 -2.29734 -57.2625 -45.846 6.15723 23.7261 2.35007 -46.7425 -35.1842 64.5372 -19.4635) 3000)

  (warning-message 3 "pose5")
  (read-line)
  (send *ri* :go-pos 0 -0.2 0)
;;  (send *ri* :angle-vector #f(1.31295 6.83724 -48.7494 84.4823 -41.7289 -5.28924 1.28871 6.90049 -49.1822 85.7578 -42.5443 -5.34816 2.9693 2.74299 5.71427 30.9983 12.4762 1.15158 45.3528 -23.5248 -4.65918 -82.9492 -28.2179 -3.21642 -56.5894 -20.7457 58.4888 68.5582 97.8562 -24.164 -18.8908 15.2464 -14.9309) 3000)

  (warning-message 3 "pose6")
  (read-line)
  (send *ri* :angle-vector #f(7.89415 -2.06685 -18.2432 40.478 -25.2288 1.1254 7.90123 -1.98295 -17.1208 40.8723 -23.1667 0.921879 3.28537 0.598671 -6.99595 20.5093 40.0 0.864953 44.2919 -23.4056 -4.24644 -80.3558 -27.8686 -2.22276 -58.233 -29.8271 0.337122 15.8251 -30.2122 -33.2757 14.9295 86.1489 -57.5938) 3000)

  (warning-message 3 "pose7")
  (read-line)
  (send *ri* :angle-vector #f(15.3824 -1.86878 -18.2996 42.1516 -22.7612 0.122319 15.2725 -1.42854 -12.0672 37.6757 -24.4884 -0.28094 9.79749 0.74218 -3.85311 28.7865 40.0 0.912593 44.8016 -23.5062 -6.01269 -79.9597 -27.5811 -0.351177 -58.2133 -5.79355 18.8694 37.8893 -23.6128 -100.309 -63.5095 43.654 19.5009) 3000) 

  (warning-message 3 "pose8")
  (read-line)
  (send *ri* :angle-vector #f(1.53283 4.90125 -26.7191 41.7163 -19.6562 -1.21719 1.45786 5.537 -24.7761 38.7354 -17.8815 -6.43836 3.09426 5.2468 -0.623484 30.9974 26.0469 1.16772 44.9097 -23.5287 -4.78767 -82.2673 -28.133 -2.78749 -56.7407 -26.2404 34.8817 58.7542 54.1795 -43.0583 5.04497 31.9651 -12.2456) 3000)

  (warning-message 3 "pose9")
  (read-line)
  (send *hand* :move-gripper :larm 0)
  )

(defun move-robot-model-for-debris3
  ()
  (let* ((base-frame "/odom") (target-frame "/debris3")
         (header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id base-frame))
         (msg (instance geometry_msgs::PoseStamped :init :header header))
         coords)
    (setq coords (send (send *tf* :lookup-transform target-frame base-frame (ros::time 0)) :inverse-transformation))
    (setq coords (send coords :transform (make-coords :pos (float-vector -451.048 993.431 -494.816)) :local))
    (send msg :pose (ros::coords->tf-pose coords))
    (ros::publish "/urdf_control_marker/set_pose" msg)
    ))

(defun move-robot-model-for-debris5
  ()
  (let* ((base-frame "/odom") (target-frame "/debris5")
         (header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id base-frame))
         (msg (instance geometry_msgs::PoseStamped :init :header header))
         coords)
    (setq coords (send (send *tf* :lookup-transform target-frame base-frame (ros::time 0)) :inverse-transformation))
    (setq coords (send coords :transform (make-coords :pos (float-vector -684.584 -285.638 -59.646)) :local))
    (send msg :pose (ros::coords->tf-pose coords))
    (ros::publish "/urdf_control_marker/set_pose" msg)
    ))

(defun main ()
  (robot-init)
  (warn ";; (insert-debris5-marker) * 2~%")
  (read-line)
  (insert-debris5-marker)
  (unix::sleep 2)
  (insert-debris5-marker)

  (warn ";; Set rviz marker and Enter~%")
  (read-line)

  (warn ";; (move-robot-model-for-debris5)~%")
  (read-line)
  (move-robot-model-for-debris5)

  (warn ";; Push Enter after go-pos !!!~%")
  (read-line)

  (warn ";; Set rviz marker and Enter~%")
  (read-line)

  (warn ";; (move-robot-model-for-debris5)~%")
  (read-line)
  (move-robot-model-for-debris5)

  (warn ";; Push Enter after go-pos !!!~%")
  (read-line)

  (warn ";; (remove-debris5)~%")
  (read-line)
  (remove-debris5)
  )
