;; left push
(((equal larm-motion-symbol :left-push)
  (larm-left-push-action :move-robot move-robot :use-torso use-torso))
 ;; front push
 ((equal larm-motion-symbol :front-push)
  (larm-front-push-action :move-robot move-robot :use-torso use-torso))
 ;; back push
 ((equal larm-motion-symbol :back-push)
  (larm-back-push-action :move-robot move-robot :use-torso use-torso))
 ;; top push
 ((equal larm-motion-symbol :top-push)
  (larm-top-push-action :move-robot move-robot :use-torso use-torso))
 ;; top-left push
 ((equal larm-motion-symbol :top-left-push)
  (larm-top-left-push-action :move-robot move-robot :use-torso use-torso))
 ;; bottom-left push
 ((equal larm-motion-symbol :bottom-left-push)
  (larm-bottom-left-push-action :move-robot move-robot :use-torso use-torso))
 ;; front-left push
 ((equal larm-motion-symbol :front-left-push)
  (larm-front-left-push-action :move-robot move-robot :use-torso use-torso))
 ;; back-left push
 ((equal larm-motion-symbol :back-left-push)
  (larm-back-left-push-action :move-robot move-robot :use-torso use-torso))
 ;; left move push
 ((equal larm-motion-symbol :left-move-push)
  (larm-left-move-push-action :move-robot move-robot :use-torso use-torso))
 ;; bottom left side push
 ((equal larm-motion-symbol :bottom-left-side-push)
  (larm-bottom-left-side-push-action :move-robot move-robot :use-torso use-torso))
 ;; top right side push
 ((equal larm-motion-symbol :top-right-side-push)
  (larm-top-right-side-push-action :move-robot move-robot :use-torso use-torso))
 ;; push release
 ((equal larm-motion-symbol :push-release)
  (larm-push-release-action :move-robot move-robot :use-torso use-torso)))
