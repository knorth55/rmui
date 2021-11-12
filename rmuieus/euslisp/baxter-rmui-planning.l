#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :human-scenes "package://rmuieus/euslisp/human/human-scenes.l")
(require :human-motions "package://rmuieus/euslisp/human/human-motions.l")
(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


;; parameters

(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)


(defun main (&key (rmui-name "rmui0"))
  (ros::roseus "baxter_rmui_planning")
  (set-rmui-parameters rmui-name)
  (scene-init :box-pos-y 0)
  (pilot-scene-init)
  (motion-init :robot *robot* :ri *ri*)
  (ros::ros-info "motion-init finished")
  (setq *server*
        (instance rmui-server :init
                  :namespace *device-namespace*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id *device-frame-id*
                  :world-frame-id *world-frame-id*
                  :motion-symbol-length *motion-symbol-length*))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (while (ros::ok) (send *server* :spin-once)))
