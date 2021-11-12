(require :human-robot "models/human-robot.l")


(defmethod human-robot
  (:reset-cube-manip-pose ()
     (send self :reset-pose)))


(provide :human-motions "human-motions.l")
