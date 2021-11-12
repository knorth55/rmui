#!/usr/bin/env roseus

(require :human-robot "models/human-robot.l")
(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


(defvar *pilot-device-id* "pilot-device")
(defvar *pilot-device* (make-cube 100 100 100 :name *pilot-device-id*))
(defvar *pilot-table-id* "room73b2-askul-1200x700-desk-0")
(defvar *pilot-table* (room73b2-askul-1200x700-desk-0))

(defvar *pilot-table-z* 0)
(defvar *pilot-table-height* 700)
(defvar *pilot-device-pos-x* 550)
(defvar *pilot-contact-offset* (float-vector -80 0 0))

(setq *pilot-device-pos-z*
      (+ (+ *pilot-table-z* *pilot-table-height*) (/ (z-of-cube *pilot-device*) 2.0)))
(setq *pilot-device-pos* (float-vector *pilot-device-pos-x* 0 *pilot-device-pos-z*))


(defun pilot-scene-init (&key (device-pos-y 0))
  (setq *pilot-table-pos*
        (float-vector (+ *pilot-device-pos-x* 100) (* -1 device-pos-y) *pilot-table-z*))
  (send *pilot-device* :reset-coords)
  (send *pilot-device* :worldcoords)
  (send *pilot-device* :newcoords (make-coords :pos *pilot-device-pos*))
  (send *pilot-device* :worldcoords)
  (send *pilot-table* :reset-coords)
  (send *pilot-table* :worldcoords)
  (send *pilot-table* :newcoords (make-coords :pos *pilot-table-pos*))
  (send *pilot-table* :worldcoords)
  (setq *human* (instance human-robot :init))
  (setq *pilot* *human*)
  ; (send *pilot* :put :larm-contact-coords
  ;       (make-cascoords :name :larm-contact-coords
  ;                       :parent (send *pilot* :link "left_wrist")
  ;                       :coords (send (send (send *pilot* :larm-end-coords) :copy-worldcoords)
  ;                                     :translate *pilot-contact-offset*)))
  ; (send *pilot* :put :rarm-contact-coords
  ;       (make-cascoords :name :rarm-contact-coords
  ;                       :parent (send *pilot* :link "right_wrist")
  ;                       :coords (send (send (send *pilot* :rarm-end-coords) :copy-worldcoords)
  ;                                     :translate *pilot-contact-offset*)))
  (setq *pilotviewer* (instance x::irtviewer :create :title "Pilot Viewer"))
  (objects (list *pilot* *pilot-device* *pilot-table*) *pilotviewer*))


(provide :human-scenes "human-scenes.l")
