#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defvar *table-z* 0)
(defvar *base-frame-id* "base_footprint")
(defvar *box-pos-x* 550)
(defvar *contact-offset* (float-vector -80 0 0))


;; functions

(defun robot-scene-init (&key (table-objects) (table-pos))
  ;; robot-init
  (setq *pr2* (instance pr2-robot :init))
  (setq *robot* *pr2*)
  (setq *ri* (instance pr2-interface :init :objects table-objects))
  (let* ((larm-contact-coords
           (send (send *robot* :larm-end-coords) :copy-worldcoords))
         (rarm-contact-coords
           (send (send *robot* :rarm-end-coords) :copy-worldcoords))
         (larm-grasp-coords
           (send (send *robot* :larm-end-coords) :copy-worldcoords))
         (rarm-grasp-coords
           (send (send *robot* :rarm-end-coords) :copy-worldcoords)))
    (robot-scene-init-base
      :table-pos table-pos
      :larm-ee-frame-id "l_wrist_roll_link"
      :rarm-ee-frame-id "r_wrist_roll_link"
      :larm-contact-coords larm-contact-coords
      :rarm-contact-coords rarm-contact-coords
      :larm-grasp-coords larm-grasp-coords
      :rarm-grasp-coords rarm-grasp-coords
      :base-frame-id *base-frame-id*)))


(provide :pr2-scenes "pr2-scenes.l")
