(require :human-robot "models/human-robot.l")


(defmethod human-robot
  ; (
  ;  :torso-waist-y :torso-waist-r :torso-waist-p :torso-chest-y :torso-chest-r :torso-chest-p ;; 6
  ;  :lleg-crotch-y :lleg-crotch-r :lleg-crotch-p :lleg-knee-p :lleg-ankle-y :lleg-ankle-p :lleg-ankle-r ;; 7
  ;  :rleg-crotch-y :rleg-crotch-r :rleg-crotch-p :rleg-knee-p :rleg-ankle-y :rleg-ankle-p :rleg-ankle-r  ;; 7
  ;  :larm-collar-y :larm-shoulder-p :larm-shoulder-r :larm-shoulder-y :larm-elbow-p :larm-wrist-y :larm-wrist-p :larm-wrist-r ;; 8
  ;  :larm-thumb-y :larm-thumb-p :larm-finger1-r1 :larm-finger1-r2 :larm-finger2-r1 :larm-finger2-r2 :larm-finger3-r1 :larm-finger3-r2 :larm-finger4-r1 :larm-finger4-r2 ;; 10 
  ;  :rarm-collar-y :rarm-shoulder-p :rarm-shoulder-r :rarm-shoulder-y :rarm-elbow-p :rarm-wrist-y :rarm-wrist-p :rarm-wrist-r  ;; 8
  ;  :rarm-thumb-y :rarm-thumb-p :rarm-finger1-r1 :rarm-finger1-r2 :rarm-finger2-r1 :rarm-finger2-r2 :rarm-finger3-r1 :rarm-finger3-r2 :rarm-finger4-r1 :rarm-finger4-r2 ;; 10
  ;  :head-neck-r :head-neck-y :head-neck-p :head-head-p :head-reye-y :head-leye-y :head-reye-p :head-leye-p ;; 8
  ;  )

  (:reset-cube-manip-pose (&optional (arm :arms))
    (let ((av (send self :angle-vector))
          (reset-av (send self :reset-pose)))
      (setq av
            (cond
              ((eq arm :arms)
               (concatenate float-vector
                            (subseq av 0 20)
                            (subseq reset-av 20 56)
                            (subseq av 56 64)))
              ((eq arm :larm)
               (concatenate float-vector
                            (subseq av 0 20)
                            (subseq reset-av 20 38)
                            (subseq av 38 64)))
              ((eq arm :rarm)
               (concatenate float-vector
                            (subseq av 0 38)
                            (subseq reset-av 38 56)
                            (subseq av 56 64)))
              (t nil)))
      (send self :angle-vector av)
      av)))


(provide :human-motions "human-motions.l")
