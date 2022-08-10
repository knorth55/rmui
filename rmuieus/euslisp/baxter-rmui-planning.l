#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :human-scenes "package://rmuieus/euslisp/human/human-scenes.l")
(require :human-motions "package://rmuieus/euslisp/human/human-motions.l")
(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")


;; parameters

;; (defvar *prx-threshold* 500)
(defvar *prx-threshold* 450)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 15)
(defvar *motion-symbol-length* 5)
(defvar *use-torso* nil)


(defun main (&key (scene-name "one_ctb") (wall-p nil) (use-grasp-mask-rcnn nil))
  (ros::roseus "baxter_rmui_planning")
  (rmui-server-main :scene-name scene-name :wall-p wall-p
                    :use-grasp-mask-rcnn use-grasp-mask-rcnn))
