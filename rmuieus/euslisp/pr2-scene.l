#!/usr/bin/env roseus

(load "models/room73b2-askul-1200x700-desk-0-object.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; parameters

(defvar *pr2* nil)
(defvar *ri* nil)
(defvar *cube-id* "cube")
; (defvar *cube* (make-cube 350 350 300 :name *cube-id*)) ;; cardboard box
(defvar *cube* (make-cube 270 420 220 :name *cube-id*))  ;; ctb
; (defvar *cube* (make-cube 300 220 500 :name *cube-id*))  ;; trash box
(defvar *desk-id* "room73b2-askul-1200x700-desk-0")
(defvar *desk* (room73b2-askul-1200x700-desk-0))
(defvar *table-z* 700)
(defvar *cube-pos-x* 550)


;; functions

(defvar *cube-pos-z* (+ *table-z* (/ (z-of-cube *cube*) 2.0)))
(defvar *cube-prev-lift-coords* nil)


(defun scene-init (&key (cube-pos-y 0))
  (send *cube* :reset-coords)
  (send *cube* :worldcoords)
  (send *cube* :newcoords
        (make-coords :pos (float-vector *cube-pos-x* 0 *cube-pos-z*)))
  (send *cube* :worldcoords)
  (send *desk* :reset-coords)
  (send *desk* :worldcoords)
  (send *desk* :newcoords
        (make-coords :pos (float-vector (+ *cube-pos-x* 100) (* -1 cube-pos-y) 0)))
  (send *desk* :worldcoords)
  (setq *pr2* (instance pr2-robot :init))
  (setq *ri* (instance pr2-interface :init :objects
                       (list (copy-object *cube*) (copy-object *desk*))))
  (objects (list *pr2* *cube* *desk*)))


(provide :pr2-scene "pr2-scene.l")
