#!/usr/bin/env roseus

(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-interface "package://baxtereus/baxter-interface.l")


(defmethod baxter-robot
  (:reset-cube-manip-pose (&optional (arm :arms))
    (let ((av (send self :angle-vector))
          (reset-av (send self :reset-pose)))
      (setq av
            (cond
              ((eq arm :arms)
               (concatenate float-vector
                            (subseq av 0 1)
                            (subseq reset-av 1 15)
                            (subseq av 15 17)))
              ((eq arm :larm)
               (concatenate float-vector
                            (subseq av 0 1)
                            (subseq reset-av 1 8)
                            (subseq av 8 17)))
              ((eq arm :rarm)
               (concatenate float-vector
                            (subseq av 0 8)
                            (subseq reset-av 8 15)
                            (subseq av 15 17)))
              (t nil)))
      (send self :angle-vector av)
      av)))


(defun start-grasp (&optional (arm :arms) (wait nil) &rest args)
  (send* *ri* :start-grasp arm :wait wait args))


(defun stop-grasp (&optional (arm :arms) (wait nil) &rest args)
  (send* *ri* :stop-grasp arm :wait wait args))


(provide :baxter-motions "baxter-motions.l")
