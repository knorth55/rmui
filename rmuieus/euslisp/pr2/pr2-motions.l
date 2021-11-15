#!/usr/bin/env roseus

(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defmethod pr2-robot
  (:reset-cube-manip-pose (&optional (arm :arms))
    (let ((av (send self :angle-vector))
          (reset-av (float-vector 300.0  ;; torso
                         ;; 75 20 85 -70 -60 -5 15 ;; larm
                         ;; -75 20 -85 -70 60 -5 -15 ;; rarm
                         ;; 70 -7.5 60 -90 -15 -5 -10  ;; larm
                         ;; -70 -7.5 -60 -90 15 -5 10  ;; rarm
                         70 30 105 -95 -30 -25 -20  ;; larm
                         -70 30 -105 -95 30 -25 20  ;; rarm
                         0.0 50.0  ;; head
                         )))
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


(defun start-grasp (&optional (arm :arms) &rest args &key (wait nil) (ri *ri*))
  (send* *ri* :start-grasp arm :objects objects args))


(defun stop-grasp (&optional (arm :arms) &rest args (wait nil) (ri *ri*))
  (send* *ri* :stop-grasp arm args))


(provide :pr2-motions "pr2-motions.l")
