#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_interactive_marker")

(load "package://hrpsys_ros_bridge_jvrc/euslisp/walking_config.l")
(load "package://drc_task_common/euslisp/primitive-marker-util.l")
(require "package://drc_task_common/euslisp/util.l")
(require "package://drc_task_common/euslisp/robot-util.l")

(setq *tf* *tfl*)
(send *ri* :start-grasp)
(ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped)

; front
(setq *a-pose* #f(-1.45948 3.57481 -36.3035 82.3102 -48.2839 5.875091e-07 -1.45966 3.58473 -38.1226 85.1258 -49.2805 -5.875092e-07 -10.1465 -2.0 32.3973 -33.0 28.6921 -8.04218 56.8175 -16.2135 -115.024 -123.466 -102.797 15.9201 -16.7946 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
(setq *b-pose* #f(-9.11524 6.06799 -29.7756 81.0597 -50.0644 5.875091e-07 -9.12446 6.1643 -35.4858 86.8277 -50.1213 -5.875092e-07 -10.8097 -1.18096 39.8766 -33.0 -15.859 -8.09635 64.2338 -35.3586 -137.787 -102.107 -96.4216 -7.86519 3.65473 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
(setq *c-pose* #f(-0.688416 1.53537 -35.1905 82.8876 -47.751 5.875092e-07 -0.688472 1.53726 -35.9658 84.0723 -48.1604 -5.875091e-07 -0.355893 -0.613609 0.65669 -33.0 -12.182 -8.29362 -72.8619 -16.0974 -10.8494 -124.705 -50.5542 21.5927 47.1768 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
(setq *d-pose* #f(-0.92578 1.3669 -35.1879 82.9025 -47.9706 5.875092e-07 -0.925646 1.36864 -35.987 83.9556 -48.2247 -5.875092e-07 -2.819 -2.0 4.86691 -33.0 -20.6054 -8.29362 -82.2898 -16.0909 -9.73854 -114.501 -38.2071 16.6569 46.4856 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))

; back
#f(-5.52645 -0.397917 -25.1332 83.2727 -51.8839 5.875092e-07 -5.53041 -0.372917 -27.4405 84.0587 -50.3627 -5.875092e-07 -3.72992 -2.0 8.19502 -33.0 -8.49327 80.8597 179.799 -15.8251 -90.6359 -56.9902 56.1184 87.6955 -57.5083 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)

(defun a-pose ()
  (send *ri* :angle-vector *a-pose*))
(defun b-pose ()
  (send *ri* :angle-vector *b-pose*))
(defun c-pose ()
  (send *ri* :angle-vector *c-pose*))
(defun d-pose ()
  (send *ri* :angle-vector *d-pose*))

(defun move-robot-with-stand-coords
  (stand-coords pos-from-ground)
  (let* ((x-vec (send (send (send pos-from-ground :worldcoords) :copy-worldcoords) :rotate-vector (float-vector 0 0 1)))
         (z-vec (float-vector 0 0 1))
         (y-vec (v* z-vec x-vec))
         (valve-coords-stand-rot (make-coords :pos (send (send *pos-from-ground* :copy-worldcoords) :worldpos) :rot (transpose (matrix x-vec y-vec z-vec))))
         (tmp-stand-coords (send (send valve-coords-stand-rot :copy-worldcoords) :transform stand-coords :local))
         (stand-pos (send tmp-stand-coords :worldpos))
         (stand-rpy (elt (send tmp-stand-coords :rpy-angle) 0))
         )
    (setq *stand-coords* (make-coords :pos (float-vector (elt stand-pos 0) (elt stand-pos 1) 0) :rpy (list (elt stand-rpy 0) 0 0)))
    )
  )
