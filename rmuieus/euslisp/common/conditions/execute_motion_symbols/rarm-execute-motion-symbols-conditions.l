;; right push
(((equal rarm-motion-symbol :right-push)
  (rarm-right-push-action :move-robot move-robot :use-torso use-torso))
 ;; front push
 ((equal rarm-motion-symbol :front-push)
  (rarm-front-push-action :move-robot move-robot :use-torso use-torso))
 ;; back push
 ((equal rarm-motion-symbol :back-push)
  (rarm-back-push-action :move-robot move-robot :use-torso use-torso))
 ;; top push
 ((equal rarm-motion-symbol :top-push)
  (rarm-top-push-action :move-robot move-robot :use-torso use-torso))
 ;; top right push
 ((equal rarm-motion-symbol :top-right-push)
  (rarm-top-right-push-action :move-robot move-robot :use-torso use-torso))
 ;; bottom right push
 ((equal rarm-motion-symbol :bottom-right-push)
  (rarm-bottom-right-push-action :move-robot move-robot :use-torso use-torso))
 ;; front right push
 ((equal rarm-motion-symbol :front-right-push)
  (rarm-front-right-push-action :move-robot move-robot :use-torso use-torso))
 ;; back right push
 ((equal rarm-motion-symbol :back-right-push)
  (rarm-back-right-push-action :move-robot move-robot :use-torso use-torso))
 ;; right move push
 ((equal rarm-motion-symbol :right-move-push)
  (rarm-right-move-push-action :move-robot move-robot :use-torso use-torso))
 ;; bottom right side push
 ((equal rarm-motion-symbol :bottom-right-side-push)
  (rarm-bottom-right-side-push-action :move-robot move-robot :use-torso use-torso))
 ;; top left side push
 ((equal rarm-motion-symbol :top-left-side-push)
  (rarm-top-left-side-push-action :move-robot move-robot :use-torso use-torso))
 ;; push release
 ((equal rarm-motion-symbol :push-release)
  (rarm-push-release-action :move-robot move-robot :use-torso use-torso)))
