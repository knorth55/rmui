#!/usr/bin/env roseus

(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")
(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-interface "package://baxtereus/baxter-interface.l")


(defmethod baxter-robot
  (:inverse-kinematics (target-coords &rest args &key (avoid-collision-distance 5) &allow-other-keys)
    ;; use raw instead
    (setq args
          (override-key-value-in-args
            args :avoid-collision-distance avoid-collision-distance))
    (send* self :inverse-kinematics-raw target-coords args))
  (:reset-cube-manip-pose (&optional (arm :arms))
    (let ((av (send self :angle-vector))
          (reset-av
            (float-vector
              0 58 0 -83 98 32 43 82
               -58 0 83 98 -32 43 -82)))
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
