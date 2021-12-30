#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :human-scenes "package://rmuieus/euslisp/human/human-scenes.l")
(require :human-motions "package://rmuieus/euslisp/human/human-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


;; 60 seconds for duration threshold
;; (defvar *duration-threshold* 60)
;; no duration wait
(defvar *duration-threshold* nil)
; (defvar *interrupt-timeout* 10)
(defvar *interrupt-timeout* 5)
(defvar *prx-threshold* 500)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 15)
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)
(defvar *use-torso* nil)


(defun main (&key (pdf nil) (scene-name "one_ctb")
                  (wall-p nil) (skip-interaction nil))
  (ros::roseus "baxter_statenet_server")
  (statenet-server-main :pdf pdf
                        :scene-name scene-name
                        :wall-p wall-p
                        :skip-interaction skip-interaction))
