;; WARNING: this file is deprecated and not used now.

;; support push hold up
(((or (and (equal larm-motion-symbol :push-hold-up)
           (equal rarm-motion-symbol :push-hold-up))
      (and (equal larm-motion-symbol :support-hold-up)
           (equal rarm-motion-symbol :support-hold-up)))
  (dualarm-hold-up-action :move-robot move-robot :use-torso use-torso))
 ;; support push hold down
 ((or (and (equal larm-motion-symbol :push-hold-down)
           (equal rarm-motion-symbol :push-hold-down))
      (and (equal larm-motion-symbol :support-hold-down)
           (equal rarm-motion-symbol :support-hold-down)))
  (dualarm-hold-down-action :move-robot move-robot :use-torso use-torso))
 ;; push rotate ccw x45
 ((and (equal larm-motion-symbol :push-rotate-ccw-x45)
       (equal rarm-motion-symbol :push-rotate-ccw-x45))
  (dualarm-push-rotate-ccw-x45-action :move-robot move-robot :use-torso use-torso))
 ;; push rotate cw x45
 ((and (equal larm-motion-symbol :push-rotate-cw-x45)
       (equal rarm-motion-symbol :push-rotate-cw-x45))
  (dualarm-push-rotate-cw-x45-action :move-robot move-robot :use-torso use-torso))
 ;; lift push rotate ccw x90
 ((or (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-ccw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :side-push-rotate-ccw-x90)))
  (dualarm-push-rotate-ccw-x90-action :move-robot move-robot :use-torso use-torso))
 ;; lift push rotate cw x90
 ((or (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-cw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
           (equal rarm-motion-symbol :side-push-rotate-cw-x90)))
  (dualarm-push-rotate-cw-x90-action :move-robot move-robot :use-torso use-torso))
 ;; push rotate ccw z45
 ((and (equal larm-motion-symbol :push-rotate-ccw-z45)
       (equal rarm-motion-symbol :push-rotate-ccw-z45))
  (dualarm-push-rotate-ccw-z45-action :move-robot move-robot :use-torso use-torso))
 ;; push rotate cw z45
 ((and (equal larm-motion-symbol :push-rotate-cw-z45)
       (equal rarm-motion-symbol :push-rotate-cw-z45))
  (dualarm-push-rotate-cw-z45-action :move-robot move-robot :use-torso use-torso))
 ;; support rotate ccw x45
 ((and (equal larm-motion-symbol :support-rotate-ccw-x45)
       (equal rarm-motion-symbol :support-rotate-ccw-x45))
  (dualarm-support-rotate-ccw-x45-action :move-robot move-robot :use-torso use-torso))
 ;; support rotate cw x45
 ((and (equal larm-motion-symbol :support-rotate-cw-x45)
       (equal rarm-motion-symbol :support-rotate-cw-x45))
  (dualarm-support-rotate-cw-x45-action :move-robot move-robot :use-torso use-torso)))