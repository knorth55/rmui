#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(setq *table-z* 0)
(setq *table-height* 700)
(setq *cube-pos-x* 550)
(setq *cube-pos-z* (+ (+ *table-z* *table-height*) (/ (z-of-cube *cube*) 2.0)))
(setq *cube-prev-lift-coords* nil)


;; functions

(defun scene-init (&key (cube-pos-y 0))
  (send *cube* :reset-coords)
  (send *cube* :worldcoords)
  (send *cube* :newcoords
        (make-coords :pos (float-vector *cube-pos-x* 0 *cube-pos-z*)))
  (send *cube* :worldcoords)
  (send *desk* :reset-coords)
  (send *desk* :worldcoords)
  (send *desk* :newcoords
        (make-coords :pos (float-vector (+ *cube-pos-x* 100) (* -1 cube-pos-y) *table-z*)))
  (send *desk* :worldcoords)
  (setq *pr2* (instance pr2-robot :init))
  (setq *robot* *pr2*)
  ;; (send (send *robot* :larm :end-coords) :translate (float-vector -80 0 30))
  ;; (send (send *robot* :rarm :end-coords) :translate (float-vector -80 0 30))
  ;; add :larm-contact-coords and :rarm-contact-coords
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "l_wrist_roll_link")
                        :coords (send (send (send *robot* :larm-end-coords) :copy-worldcoords)
                                      :translate (float-vector -80 0 30))))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "r_wrist_roll_link")
                        :coords (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                                      :translate (float-vector -80 0 30))))
  (setq *ri* (instance pr2-interface :init :objects
                       (list (copy-object *cube*) (copy-object *desk*))))
  (objects (list *pr2* *cube* *desk*)))


(provide :pr2-scenes "pr2-scenes.l")
