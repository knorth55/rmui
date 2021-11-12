#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defvar *table-z* 0)
(defvar *table-height* 700)
(defvar *box-pos-x* 550)
(defvar *contact-offset* (float-vector -80 0 0))

(setq *box-prev-lift-coords* nil)
(setq *box-pos-z* (+ (+ *table-z* *table-height*) (/ (z-of-cube *box*) 2.0)))
(setq *box-pos* (float-vector *box-pos-x* 0 *box-pos-z*))


;; functions

(defun scene-init (&key (box-pos-y 0))
  (setq *table-pos* (float-vector (+ *box-pos-x* 100) (* -1 box-pos-y) *table-z*))
  (send *box* :reset-coords)
  (send *box* :worldcoords)
  (send *box* :newcoords
        (make-coords :pos (float-vector *box-pos-x* 0 *box-pos-z*)))
  (send *box* :worldcoords)
  (send *table* :reset-coords)
  (send *table* :worldcoords)
  (send *table* :newcoords
        (make-coords :pos (float-vector (+ *box-pos-x* 100) (* -1 box-pos-y) *table-z*)))
  (send *table* :worldcoords)
  (setq *pr2* (instance pr2-robot :init))
  (setq *robot* *pr2*)
  (setq *ri* (instance pr2-interface :init :objects
                       (list (copy-object *box*) (copy-object *table*))))
  ;; (send (send *robot* :larm :end-coords) :translate *contact-offset*)
  ;; (send (send *robot* :rarm :end-coords) :translate *contact-offset*)
  ;; add :larm-contact-coords and :rarm-contact-coords
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "l_wrist_roll_link")
                        :coords (send (send (send *robot* :larm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "r_wrist_roll_link")
                        :coords (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send (send *ri* :robot) :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send (send *ri* :robot) :link "l_wrist_roll_link")
                        :coords
                        (send (send (send (send *ri* :robot) :larm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (send (send *ri* :robot) :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send (send *ri* :robot) :link "r_wrist_roll_link")
                        :coords
                        (send (send (send (send *ri* :robot) :rarm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (make-irtviewer :title "Robot Viewer")
  (objects (list *pr2* *box* *table*) *irtviewer*))


(provide :pr2-scenes "pr2-scenes.l")
