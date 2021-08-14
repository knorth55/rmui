(((equal larm-motion-symbol :left-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :left)
               :approach-directions (list :back)
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :front-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list :left)
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :back-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list :left)
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :top-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list :left)
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :top-left-push)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :bottom-left-push)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :front-left-push)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :back-left-push)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :left-move-push)
  (assoc-cube :larm)
  (push-move-motion :larm :move-robot move-robot :use-torso use-torso
                    :push-axis :y
                    :move-distance -200.0)
  (dissoc-cube :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :bottom-left-side-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :top-right-side-push)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :larm)
  (return-from execute-motion-symbols (list larm-motion-symbol nil)))
 ((equal larm-motion-symbol :push-release)
  (push-release-motion :larm :move-robot move-robot :use-torso use-torso)
  (reset-cube-manip-motion :larm :move-robot move-robot)
  (return-from execute-motion-symbols (list larm-motion-symbol nil))))
