#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :common-rmui-planner "package://rmuieus/euslisp/common/common-rmui-planner.l")


;; parameters

(defvar *namespace* "/rmui0/rmui_node")
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
(defvar *device-frame-id* "rmui_link")
(defvar *world-frame-id* "world")
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)


(defun main ()
  (ros::roseus "baxter_rmui_planning")
  (motion-init)
  (ros::ros-info "motion-init finished")
  (setq *planner*
        (instance rmui-planner :init
                  :namespace *namespace*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id *device-frame-id*
                  :world-frame-id *world-frame-id*
                  :motion-symbol-length *motion-symbol-length*))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (ros::create-timer 0.1 #'send *planner* :planner-cb)
  (ros::spin))
