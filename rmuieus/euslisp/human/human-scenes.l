#!/usr/bin/env roseus

(require :human-robot "models/human-robot.l")
(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


(defvar *pilot-table-z* 0)
(defvar *pilot-table-height* 700)
(defvar *pilot-device-pos-x* 550)
(defvar *pilot-contact-offset* (float-vector 30 0 20))


(defun pilot-robot-scene-init  (&key (table-pos))
  (setq *human* (instance human-robot :init))
  (setq *pilot* *human*)
  (send (send *pilot* :larm-end-coords) :rotate pi/2 :y)
  (send (send *pilot* :larm-end-coords) :rotate pi/2 :x)
  (send (send *pilot* :rarm-end-coords) :rotate pi/2 :y)
  (send (send *pilot* :rarm-end-coords) :rotate -pi/2 :x)
  (pilot-robot-scene-init-base :table-pos table-pos
                               :larm-ee-frame-id :larm-human-wrist-roll
                               :rarm-ee-frame-id :rarm-human-wrist-roll))


(provide :human-scenes "human-scenes.l")
