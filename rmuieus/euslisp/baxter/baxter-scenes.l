#!/usr/bin/env roseus

(require :baxter-interface "package://baxtereus/baxter-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(setq *table-z* -600)
(setq *table-height* 700)
(setq *cube-pos-x* 700)
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
  (setq *baxter* (instance baxter-robot :init))
  (setq *robot* *baxter*)
  (setq *ri* (instance baxter-interface :init :objects
                       (list (copy-object *cube*) (copy-object *desk*))))
  (objects (list *baxter* *cube* *desk*)))


(provide :baxter-scene "baxter-scene.l")
