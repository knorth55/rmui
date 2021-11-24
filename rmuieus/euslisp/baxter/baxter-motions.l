#!/usr/bin/env roseus

(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")
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


(defun get-arm-controller (arm &key (use-torso t))
  (cond
    ((eq arm :larm) :larm-controller)
    ((eq arm :rarm) :rarm-controller)
    ((eq arm :arms) :default-controller)))


(defun start-grasp (&optional (arm :arms) &rest args &key (wait nil) (ri *ri*))
  (setq args (remove-key-from-args args :ri))
  (send* ri :start-grasp arm :wait wait args))


(defun stop-grasp (&optional (arm :arms) &rest args (wait nil) (ri *ri*))
  (setq args (remove-key-from-args args :ri))
  (send* ri :stop-grasp arm :wait wait args))


(provide :baxter-motions "baxter-motions.l")
