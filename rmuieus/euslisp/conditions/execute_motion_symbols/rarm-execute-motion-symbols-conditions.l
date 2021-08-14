(((equal rarm-motion-symbol :right-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :right)
               :approach-directions (list :back)
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :front-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list :right)
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :back-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list :right)
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :top-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list :right)
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :top-right-push)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :bottom-right-push)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :front-right-push)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :back-right-push)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :right-move-push)
  (assoc-cube :rarm)
  (push-move-motion :rarm
                    :push-axis :y
                    :move-distance 200.0
                    :move-robot move-robot
                    :use-torso use-torso)
  (dissoc-cube :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :bottom-right-side-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :top-left-side-push)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :rarm)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
 ((equal rarm-motion-symbol :push-release)
  (push-release-motion :rarm :move-robot move-robot :use-torso use-torso)
  (reset-cube-manip-motion :rarm :move-robot move-robot)
  (return-from execute-motion-symbols (list nil rarm-motion-symbol))))
