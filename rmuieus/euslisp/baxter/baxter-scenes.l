#!/usr/bin/env roseus

(require :baxter-interface "package://baxtereus/baxter-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(setq *table-z* -900)
(setq *table-height* 700)
(setq *cube-pos-x* 600)
(setq *contact-offset* (float-vector -150 0 30))

(setq *cube-prev-lift-coords* nil)
(setq *cube-pos-z* (+ (+ *table-z* *table-height*) (/ (z-of-cube *cube*) 2.0)))
(setq *cube-pos* (float-vector *cube-pos-x* 0 *cube-pos-z*))


;; functions

(defun scene-init (&key (cube-pos-y 0))
  (setq *desk-pos* (float-vector (+ *cube-pos-x* 100) (* -1 cube-pos-y) *table-z*))
  (send *cube* :reset-coords)
  (send *cube* :worldcoords)
  (send *cube* :newcoords (make-coords :pos *cube-pos*))
  (send *cube* :worldcoords)
  (send *desk* :reset-coords)
  (send *desk* :worldcoords)
  (send *desk* :newcoords (make-coords :pos *desk-pos*))
  (send *desk* :worldcoords)
  (setq *baxter* (instance baxter-robot-safe :init))
  (setq *robot* *baxter*)
  (setq *ri* (instance baxter-interface :init :objects
                       (list (copy-object *cube*) (copy-object *desk*))))
  ;; (send (send *robot* :larm :end-coords) :translate *contact-offset*)
  ;; (send (send *robot* :rarm :end-coords) :translate *contact-offset*)
  ;; add :larm-contact-coords and :rarm-contact-coords
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "left_wrist")
                        :coords (send (send (send *robot* :larm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "right_wrist")
                        :coords (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send (send *ri* :robot) :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send (send *ri* :robot) :link "left_wrist")
                        :coords
                        (send (send (send (send *ri* :robot) :larm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (send (send *ri* :robot) :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send (send *ri* :robot) :link "right_wrist")
                        :coords
                        (send (send (send (send *ri* :robot) :rarm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (objects (list *baxter* *cube* *desk*)))


(provide :baxter-scene "baxter-scene.l")
