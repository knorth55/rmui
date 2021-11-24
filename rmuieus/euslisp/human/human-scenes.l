#!/usr/bin/env roseus

(require :human-robot "models/human-robot.l")
(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


(defvar *pilot-table-z* 0)
(defvar *pilot-device-pos-x* 550)
(defvar *pilot-contact-offset* (float-vector 30 0 20))


(defun pilot-robot-scene-init (&key (table-pos))
  (setq *human* (instance human-robot :init))
  (setq *pilot* *human*)
  (let* ((larm-contact-coords
           (send (send *pilot* :larm-end-coords) :copy-worldcoords))
         (rarm-contact-coords
           (send (send *pilot* :rarm-end-coords) :copy-worldcoords)))
    (send larm-contact-coords :rotate pi/2 :y)
    (send larm-contact-coords :rotate pi/2 :x)
    (send rarm-contact-coords :rotate pi/2 :y)
    (send rarm-contact-coords :rotate -pi/2 :x)
    (pilot-robot-scene-init-base
      :table-pos table-pos
      :larm-ee-frame-id :larm-human-wrist-roll
      :rarm-ee-frame-id :rarm-human-wrist-roll
      :larm-contact-coords larm-contact-coords
      :rarm-contact-coords rarm-contact-coords)))


(provide :human-scenes "human-scenes.l")
