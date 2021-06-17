#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :common-scene "package://rmuieus/euslisp/common/common-scene.l")


(setq *pr2* nil)
(setq *ri* nil)

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
        (make-coords :pos (float-vector (+ *cube-pos-x* 100) (* -1 cube-pos-y) 0)))
  (send *desk* :worldcoords)
  (setq *pr2* (instance pr2-robot :init))
  (setq *ri* (instance pr2-interface :init :objects
                       (list (copy-object *cube*) (copy-object *desk*))))
  (objects (list *pr2* *cube* *desk*)))


(provide :pr2-scene "pr2-scene.l")
