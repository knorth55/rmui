;; (require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")


;; init

(defun motion-init (&key (move-robot t)
                         (robot *robot*)
                         (ri *ri*)
                         (cubes *boxes*)
                         (vw *irtviewer*))
  (dolist (cube cubes)
    (add-cube-coords :cube cube)
    ;; TODO: support cube collision avoidance
    ; (add-cube-collision-object :cube cube :robot robot :ri ri)
    )
  (reset-motion :robot robot :ri ri :move-robot move-robot :vw vw))


(defun get-grasp-coords (arm &key (robot *robot*))
  (cond ((eq arm :larm) (send robot :get :larm-grasp-coords))
        ((eq arm :rarm) (send robot :get :rarm-grasp-coords))
        ((eq arm :arms)
         (list (get-grasp-coords :larm :robot robot)
               (get-grasp-coords :rarm :robot robot)))
        (t nil)))


(defun get-contact-coords (arm &key (robot *robot*))
  (cond ((eq arm :larm) (send robot :get :larm-contact-coords))
        ((eq arm :rarm) (send robot :get :rarm-contact-coords))
        ((eq arm :arms)
         (list (get-contact-coords :larm :robot robot)
               (get-contact-coords :rarm :robot robot)))
        (t nil)))


(defun get-ee-coords (arm &key (ee-type :contact) (robot *robot*))
  (cond ((equal ee-type :contact)
         (get-contact-coords arm :robot robot))
        ((equal ee-type :grasp)
         (get-grasp-coords arm :robot robot))
        (t nil)))


(defun get-ee-link-list (arm &key (use-torso t) (ee-type :contact) (robot *robot*))
  (send robot :link-list
        (send (get-ee-coords arm :ee-type ee-type :robot robot) :parent)
        (if (not use-torso) (car (send robot arm)))))


(defun get-ee-ik-args
  (arm &key (rotation-axis t) (use-torso t) (ee-type :contact) (robot *robot*))
  (if (eq arm :arms)
    (list
      :move-target (list (get-ee-coords :larm :ee-type ee-type :robot robot)
                         (get-ee-coords :rarm :ee-type ee-type :robot robot))
      :link-list (list (get-ee-link-list :larm
                                         :use-torso use-torso
                                         :ee-type ee-type
                                         :robot robot)
                       (get-ee-link-list :rarm
                                         :use-torso use-torso
                                         :ee-type ee-type
                                         :robot robot))
      :rotation-axis (list rotation-axis rotation-axis)
      :use-torso use-torso)
    (list
      :move-target (list (get-ee-coords arm :ee-type ee-type :robot robot))
      :link-list (list (get-ee-link-list arm
                                         :use-torso use-torso
                                         :ee-type ee-type
                                         :robot robot))
      :rotation-axis (list rotation-axis)
      :use-torso use-torso)))


(defun solve-ik
  (arm target-coords &key (rotation-axis t) (use-torso t) (ee-type :contact) (robot *robot*))
  (send* robot :inverse-kinematics
         (cond
           ((eq arm :arms) target-coords)
           ((eq arm :larm)
            (list target-coords
                  (send (get-ee-coords :rarm :ee-type ee-type :robot robot)
                        :copy-worldcoords)))
           ((eq arm :rarm)
            (list (send (get-ee-coords :larm :ee-type ee-type :robot robot)
                        :copy-worldcoords)
                  target-coords))
           (t nil))
         (get-ee-ik-args :arms
                         :rotation-axis rotation-axis
                         :use-torso use-torso
                         :ee-type ee-type
                         :robot robot)))


(defun solve-ik-rotation-relax
  (arm target-coords &key (use-torso t) (ee-type :contact) (robot *robot*))
  (let (av)
    (setq av (solve-ik arm target-coords
                       :rotation-axis t :use-torso use-torso :ee-type ee-type
                       :robot robot))
    (if (not av)
      (setq av (solve-ik arm target-coords
                         :rotation-axis :z :use-torso use-torso :ee-type ee-type
                         :robot robot)))
    av))


(defun grasp-cube (arm &key (cube *box0*) (robot *robot*))
  (if (eq arm :arms) (return-from grasp-cube nil))
  (let* ((coords-name (if (eq arm :larm) :larm-grasp-coords :rarm-grasp-coords))
         (grasp-coords
           (make-cascoords :name coords-name
                           :coords (send (get-grasp-coords arm :robot robot)
                                         :copy-worldcoords))))
    (send cube :assoc grasp-coords)
    (send cube :put coords-name grasp-coords)))


(defun ungrasp-cube (arm &key (cube *box0*))
  (if (eq arm :arms) (return-from ungrasp-cube nil))
  (let ((coords-name (if (eq arm :larm) :larm-grasp-coords :rarm-grasp-coords)))
    (send cube :dissoc (send cube :get coords-name))
    (send cube :put coords-name nil)))


(defun contact-cube (arm &key (cube *box0*) (robot *robot*))
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (contact-cube-step a :cube cube :robot robot)))


(defun contact-cube-step (arm &key (cube *box0*) (robot *robot*))
  (let* ((coords-name (if (eq arm :larm) :larm-contact-coords :rarm-contact-coords))
         (contact-coords
           (make-cascoords :name coords-name
                           :coords (send (get-contact-coords arm :robot robot)
                                         :copy-worldcoords))))
    (send cube :assoc contact-coords)
    (send cube :put coords-name contact-coords)))


(defun discontact-cube (arm &key (cube *box0*))
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (discontact-cube-step a :cube cube)))


(defun discontact-cube-step (arm &key (cube *box0*))
  (let ((coords-name (if (eq arm :larm) :larm-contact-coords :rarm-contact-coords)))
    (send cube :dissoc (send cube :get coords-name))
    (send cube :put coords-name nil)))


(defun assoc-cube (arm &key (ee-type :contact) (ri *ri*) (cube *box0*))
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (assoc-cube-step a :ee-type ee-type :ri ri :cube cube)))


(defun assoc-cube-step (arm &key (ee-type :contact) (ri *ri*) (cube *box0*))
  (if (and ri (send ri :simulation-modep))
    (send (get-ee-coords arm :ee-type ee-type :robot (send ri :robot))
          :assoc (car (send ri :find-object (send cube :name))))))


(defun dissoc-cube (arm &key (ee-type :contact) (ri *ri*) (cube *box0*))
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (dissoc-cube-step a :ee-type ee-type :ri ri :cube cube)))


(defun dissoc-cube-step (arm &key (ee-type :contact) (ri *ri*) (cube *box0*))
  (if (and ri (send ri :simulation-modep))
    (send (get-ee-coords arm :ee-type ee-type :robot (send ri :robot))
          :dissoc (car (send ri :find-object (send cube :name))))))


(defun assoc-two-cubes (cube target-cube &key (ri *ri*))
  (send cube :assoc target-cube)
  (if (and ri (send ri :simulation-modep))
    (send (car (send ri :find-object (send cube :name)))
          :assoc (car (send ri :find-object (send target-cube :name))))))


(defun dissoc-two-cubes (cube target-cube &key (ri *ri*))
  (send cube :dissoc target-cube)
  (if (and ri (send ri :simulation-modep))
    (send (car (send ri :find-object (send cube :name)))
          :dissoc (car (send ri :find-object (send target-cube :name))))))


(defun add-cube-coords (&key (cube *box0*))
  (let* ((cube-depth (x-of-cube cube))
         (cube-width (y-of-cube cube))
         (cube-height (z-of-cube cube))
         (cube-coords (send (send cube :worldcoords) :copy-worldcoords))
         (front-cube-coords
           (make-cascoords :name :front-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector (/ cube-depth 2.0) 0 0) :world)))
         (back-cube-coords
           (make-cascoords :name :back-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector (/ cube-depth -2.0) 0 0) :world)))
         (left-cube-coords
           (make-cascoords :name :left-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector 0 (/ cube-width 2.0) 0) :world)))
         (right-cube-coords
           (make-cascoords :name :right-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector 0 (/ cube-width -2.0) 0) :world)))
         (top-cube-coords
           (make-cascoords :name :top-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector 0 0 (/ cube-height 2.0)) :world)))
         (bottom-cube-coords
           (make-cascoords :name :bottom-cube-coords
                           :coords
                           (send (send cube-coords :copy-worldcoords)
                                 :translate (float-vector 0 0 (/ cube-height -2.0)) :world))))
    (send cube :assoc front-cube-coords)
    (send cube :assoc back-cube-coords)
    (send cube :assoc left-cube-coords)
    (send cube :assoc right-cube-coords)
    (send cube :assoc top-cube-coords)
    (send cube :assoc bottom-cube-coords)
    (send cube :put :front-cube-coords front-cube-coords)
    (send cube :put :back-cube-coords back-cube-coords)
    (send cube :put :left-cube-coords left-cube-coords)
    (send cube :put :right-cube-coords right-cube-coords)
    (send cube :put :top-cube-coords top-cube-coords)
    (send cube :put :bottom-cube-coords bottom-cube-coords)))


(defun add-cube-collision-object (&key (cube *box0*) (robot *robot*) (ri *ri*))
  (if (and ri (not (send ri :simulation-modep)))
    (progn
      (send *co* :add-object cube
            :frame-id (send robot :get :base-frame-id)
            :relative-pose (send (send cube :worldcoords) :copy-worldcoords)
            :object-id (send cube :name))
      (unix::usleep (round (* 0.1 1e6))))))


(defun add-cubes-collision-object (&key (cubes *boxes*) (robot *robot*) (ri *ri*))
  (dolist (cb cubes) (add-cube-collision-object :cube cb :robot robot :ri ri)))


(defun delete-cube-collision-object (&key (cube *box0*) (ri *ri*))
  (if (and ri (not (send ri :simulation-modep)))
    (send *co* :delete-object cube)))


(defun delete-cubes-collision-object (&key (cubes *boxes*) (ri *ri*))
  (dolist (cb cubes) (delete-cube-collision-object :cube cb :ri ri)))


(defun get-cube-length (axis &key (cube-rpy (float-vector 0 0 0)) (cube *box0*))
  (let* ((inverse-coords (send (make-coords :rpy cube-rpy) :inverse-transformation))
         (cube-cpos (send (send cube :copy-worldcoords) :worldpos))
         (cube-coords
           (list (send (send cube :get :front-cube-coords) :copy-worldcoords)
                 (send (send cube :get :back-cube-coords) :copy-worldcoords)
                 (send (send cube :get :left-cube-coords) :copy-worldcoords)
                 (send (send cube :get :right-cube-coords) :copy-worldcoords)
                 (send (send cube :get :top-cube-coords) :copy-worldcoords)
                 (send (send cube :get :bottom-cube-coords) :copy-worldcoords)))
         (cube-pos-list
           (mapcar
             #'(lambda (c)
                 (elt (send inverse-coords :rotate-vector (v- (send c :worldpos) cube-cpos))
                      (cond ((eq axis :x) 0) ((eq axis :y) 1) ((eq axis :z) 2) (t nil))))
             cube-coords)))
    (- (reduce #'max cube-pos-list) (reduce #'min cube-pos-list))))


(defun get-cube-depth (&key (cube-rpy (float-vector 0 0 0)) (cube *box0*))
  (get-cube-length :x :cube-rpy cube-rpy :cube cube))


(defun get-cube-width (&key (cube-rpy (float-vector 0 0 0)) (cube *box0*))
  (get-cube-length :y :cube-rpy cube-rpy :cube cube))


(defun get-cube-height (&key (cube-rpy (float-vector 0 0 0)) (cube *box0*))
  (get-cube-length :z :cube-rpy cube-rpy :cube cube))


(defun get-cube-coords (direction &key (cube *box0*))
  (let* ((cube-coords
           (list (send (send cube :get :front-cube-coords) :copy-worldcoords)
                 (send (send cube :get :back-cube-coords) :copy-worldcoords)
                 (send (send cube :get :left-cube-coords) :copy-worldcoords)
                 (send (send cube :get :right-cube-coords) :copy-worldcoords)
                 (send (send cube :get :top-cube-coords) :copy-worldcoords)
                 (send (send cube :get :bottom-cube-coords) :copy-worldcoords)))
         (cube-pos-list
           (mapcar #'(lambda (c)
                       (elt (send c :worldpos)
                            (cond ((or (eq direction :front) (eq direction :back)) 0)
                                  ((or (eq direction :left) (eq direction :right)) 1)
                                  ((or (eq direction :top) (eq direction :bottom)) 2)
                                  (t nil))))
                   cube-coords))
         (eval-func
           (cond ((or (eq direction :front) (eq direction :left) (eq direction :top)) #'max)
                 ((or (eq direction :back) (eq direction :right) (eq direction :bottom)) #'min)
                 (t nil)))
         (eval-value (reduce eval-func cube-pos-list))
         (coords-index (position-if #'(lambda (x) (eq x eval-value)) cube-pos-list)))
    (elt cube-coords coords-index)))


(defun get-front-cube-coords (&key (cube *box0*))
  (get-cube-coords :front :cube cube))


(defun get-back-cube-coords (&key (cube *box0*))
  (get-cube-coords :back :cube cube))


(defun get-left-cube-coords (&key (cube *box0*))
  (get-cube-coords :left :cube cube))


(defun get-right-cube-coords (&key (cube *box0*))
  (get-cube-coords :right :cube cube))


(defun get-top-cube-coords (&key (cube *box0*))
  (get-cube-coords :top :cube cube))


(defun get-bottom-cube-coords (&key (cube *box0*))
  (get-cube-coords :bottom :cube cube))


(defun get-ee-cube-coords (arm &key (ee-type :contact) (cube *box0*))
  (cond ((equal ee-type :contact)
         (get-contact-cube-coords arm :cube cube))
        ((equal ee-type :grasp)
         (get-grasp-cube-coords arm :cube cube))
        (t nil)))


(defun get-contact-cube-coords (arm &key (cube *box0*))
  (cond ((eq arm :larm) (send cube :get :larm-contact-coords))
        ((eq arm :rarm) (send cube :get :rarm-contact-coords))
        ((eq arm :arms)
         (list (get-contact-cube-coords :larm :cube cube)
               (get-contact-cube-coords :rarm :cube cube)))
        (t nil)))


(defun get-grasp-cube-coords (arm &key (cube *box0*))
  (cond ((eq arm :larm) (send cube :get :larm-grasp-coords))
        ((eq arm :rarm) (send cube :get :rarm-grasp-coords))
        ((eq arm :arms)
         (list (get-grasp-cube-coords :larm :cube cube)
               (get-grasp-cube-coords :rarm :cube cube)))
        (t nil)))


;; motions
;; push motion

(defun get-push-target-coords
  (arm &key (motion-type :push)
            (push-axis :y)
            (push-position :center)
            (push-direction :left)
            (approach-direction :back)
            (hold-type :tight)
            (cube-pos nil)
            (cube-rpy (float-vector 0 0 0))
            (cube-height nil)
            (cube-width nil)
            (cube-depth nil)
            (cube *box0*))
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq arm :larm) (eq arm :rarm))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from get-push-target-coords nil))
  (unless (or (eq push-axis :x) (eq push-axis :y) (eq push-axis :z))
    (ros::ros-error (format nil "push-axis is invalid: ~A" push-axis))
    (return-from get-push-target-coords nil))
  (unless (or (eq hold-type :tight) (eq hold-type :normal) (eq hold-type :light)
              (numberp hold-type))
    (ros::ros-error (format nil "hold-type is invalid: ~A" hold-type))
    (return-from get-push-target-coords nil))
  (when (or (and (eq push-axis :x)
                 (not (and (or (eq push-direction :front)
                               (eq push-direction :back))
                           (or (and (or (and (eq arm :larm) (eq approach-direction :left))
                                        (and (eq arm :rarm) (eq approach-direction :right)))
                                    (or (eq push-position :top)
                                        (eq push-position :center)
                                        (eq push-position :bottom)))
                               (and (eq approach-direction :top)
                                    (or (eq push-position :left)
                                        (eq push-position :center)
                                        (eq push-position :right)))))))
            (and (eq push-axis :y)
                 (not (and (or (eq push-direction :left)
                               (eq push-direction :right))
                           (or (and (eq approach-direction :back)
                                    (or (eq push-position :top)
                                        (eq push-position :center)
                                        (eq push-position :bottom)))
                               (and (eq approach-direction :top)
                                    (or (eq push-position :front)
                                        (eq push-position :center)
                                        (eq push-position :back)))))))
            (and (eq push-axis :z)
                 (not (and (or (eq push-direction :top)
                               (eq push-direction :bottom))
                           (or (and (eq approach-direction :back)
                                    (or (eq push-position :left)
                                        (eq push-position :center)
                                        (eq push-position :right)))
                               (and (or (and (eq arm :larm) (eq approach-direction :left))
                                        (and (eq arm :rarm) (eq approach-direction :right)))
                                    (or (eq push-position :front)
                                        (eq push-position :center)
                                        (eq push-position :back))))))))
    (ros::ros-error "argument combination is invalid")
    (ros::ros-error (format nil "arm: ~A" arm))
    (ros::ros-error (format nil "push-axis: ~A" push-axis))
    (ros::ros-error (format nil "push-position: ~A" push-position))
    (ros::ros-error (format nil "push-direction: ~A" push-direction))
    (ros::ros-error (format nil "approach-direction: ~A" approach-direction))
    (return-from get-push-target-coords nil))
  (let ((target-coords (make-coords :pos (copy-seq cube-pos) :rpy (copy-seq cube-rpy)))
        (hold-offset (cond ((eq hold-type :tight) 50)
                           ((eq hold-type :normal) 0)
                           ((eq hold-type :light) -50)
                           ((numberp hold-type) hold-type)
                           (t nil))))
    (cond
      ((eq push-axis :x)
       (let ((x-offset
               (* (if (eq push-direction :front) 1.0 -1.0)
                  (+ (/ cube-depth 2.0) (if (eq motion-type :approach) 100 0)))))
         (cond
           ((or (and (eq arm :larm) (eq approach-direction :left))
                (and (eq arm :rarm) (eq approach-direction :right)))
            (let ((y-offset (* (if (eq arm :larm) -1.0 1.0) hold-offset))
                  (z-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :top) (- (/ cube-height 2.0) 50))
                                  ((eq push-position :bottom) (- 50 (/ cube-height 2.0))))))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              (send target-coords :rotate (if (eq push-direction :front) -pi/2 pi/2) :y)
              (send target-coords :rotate (if (eq arm :larm) -pi/2 pi/2) :z)))
           ((eq approach-direction :top)
            (let ((y-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :left)
                                   (- (/ cube-width 2.0) 50))
                                  ((eq push-position :right)
                                   (- 50 (/ cube-width 2.0)))))
                  (z-offset (* -1.0 hold-offset)))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              (send target-coords :rotate pi/2 :y)
              (when (eq push-direction :front) (send target-coords :rotate pi :x))))
           (t (return-from get-push-target-coords nil)))
         target-coords))
      ((eq push-axis :y)
       (let ((y-offset
               (* (if (eq push-direction :left) 1.0 -1.0)
                  (+ (/ cube-width 2.0) (if (eq motion-type :approach) 100 0)))))
         (cond
           ((eq approach-direction :back)
            (let ((x-offset hold-offset)
                  (z-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :top)
                                   (- (/ cube-height 2.0) 50))
                                  ((eq push-position :bottom)
                                   (- 50 (/ cube-height 2.0))))))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)))
           ((eq approach-direction :top)
            (let ((x-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :front)
                                   (- (/ cube-depth 2.0) 50))
                                  ((eq push-position :back)
                                   (- 50 (/ cube-depth 2.0)))))
                  (z-offset (* -1.0 hold-offset)))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)
              (send target-coords :rotate (if (eq push-direction :left) -pi/2 pi/2) :z)))
           (t (return-from get-push-target-coords nil)))
         target-coords))
      ((eq push-axis :z)
       (let ((z-offset
               (* (if (eq push-direction :top) 1.0 -1.0)
                  (+ (/ cube-height 2.0)
                     (if (eq motion-type :approach)
                       (if (eq push-direction :top) 50 100) 0)))))
         (cond
           ((eq approach-direction :back)
            (let ((x-offset hold-offset)
                  (y-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :left)
                                   (+ (- (/ cube-width 2.0) 50)
                                      (if (eq motion-type :approach) 100 0)))
                                  ((eq push-position :right)
                                   (+ (- 50 (/ cube-width 2.0))
                                      (if (eq motion-type :approach) -100 0))))))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              (when (eq push-direction :top) (send target-coords :rotate pi :x))))
           ((or (and (eq arm :larm) (eq approach-direction :left))
                (and (eq arm :rarm) (eq approach-direction :right)))
            (let ((x-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :front)
                                   (- (/ cube-depth 2.0) 50))
                                  ((eq push-position :back)
                                   (- 50 (/ cube-depth 2.0)))))
                  (y-offset (* (if (eq arm :larm) -1.0 1.0)
                               (+ (if (eq motion-type :approach) -100 0)
                                  hold-offset))))
              (send target-coords :translate (float-vector x-offset y-offset z-offset))
              ;; correct ???
              (send target-coords :rotate (* (if (eq arm :larm) 1.0 -1.0) pi/2) :z)
              (when (eq push-direction :top) (send target-coords :rotate pi :y))))
           (t (return-from get-push-target-coords nil)))
         target-coords))
      (t (return-from get-push-target-coords nil)))))


(defun push-motion-step
  (arm &key (motion-type :push)
            (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (hold-type :tight)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001)
            (cube-pos nil)
            (cube-rpy (float-vector 0 0 0))
            (cube-height nil)
            (cube-width nil)
            (cube-depth nil)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (vw *irtviewer*))
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq arm :larm) (eq arm :rarm) (eq arm :arms))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from push-motion-step nil))
  ;; push
  (let ((av))
    (if (eq arm :arms)
      (let ((larm-target-coords
              (get-push-target-coords
                :larm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions)
                :hold-type hold-type
                :cube-pos cube-pos
                :cube-rpy cube-rpy
                :cube-height cube-height
                :cube-width cube-width
                :cube-depth cube-depth
                :cube cube))
            (rarm-target-coords
              (get-push-target-coords
                :rarm :motion-type motion-type
                :push-axis (cadr push-axes)
                :push-position (cadr push-positions)
                :push-direction (cadr push-directions)
                :approach-direction (cadr approach-directions)
                :hold-type hold-type
                :cube-pos cube-pos
                :cube-rpy cube-rpy
                :cube-height cube-height
                :cube-width cube-width
                :cube-depth cube-depth
                :cube cube)))
        (unless (and larm-target-coords rarm-target-coords)
          (ros::ros-error "larm target coords is invalid: ~A" larm-target-coords)
          (ros::ros-error "rarm target coords is invalid: ~A" rarm-target-coords)
          (return-from push-motion-step nil))
        (setq av
              (solve-ik-rotation-relax
                :arms (list larm-target-coords rarm-target-coords)
                :use-torso use-torso :robot robot)))
      (let ((target-coords
              (get-push-target-coords
                arm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions)
                :hold-type hold-type
                :cube-pos cube-pos
                :cube-rpy cube-rpy
                :cube-height cube-height
                :cube-width cube-width
                :cube-depth cube-depth
                :cube cube)))
        (unless target-coords
          (return-from push-motion-step nil))
        (setq av
              (solve-ik-rotation-relax
                arm target-coords
                :use-torso use-torso :robot robot))))
    (when (and av move-robot)
      (send ri :angle-vector av tm
            (get-arm-controller :arms :use-torso use-torso) 0)
      (send ri :wait-interpolation))
    (send vw :draw-objects)
    (when av (list (list av) (list tm)))))


(defun push-motion
  (arm &rest args
       &key (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (approach-arm nil)
            (hold-type :tight)
            (move-robot t)
            (push-tm 3000)
            (approach-tm 3000)
            (use-torso 0.001)
            (cube-pos nil)
            (cube-rpy (float-vector 0 0 0))
            (cube-height nil)
            (cube-width nil)
            (cube-depth nil)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (cubes *boxes*)
            (vw *irtviewer*)
            &allow-other-keys)
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq arm :larm) (eq arm :rarm) (eq arm :arms))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from push-motion nil))
  (let* ((approached)
         (approach-arm (if approach-arm approach-arm arm))
         (approach-push-axes
           (cond ((and (eq arm :arms) (not (eq approach-arm :arms)))
                  (if (eq approach-arm :larm)
                    (list (car push-axes))
                    (cdr push-axes)))
                 (t push-axes)))
         (approach-push-positions
           (cond ((and (eq arm :arms) (not (eq approach-arm :arms)))
                  (if (eq approach-arm :larm)
                    (list (car push-positions))
                    (cdr push-positions)))
                 (t push-positions)))
         (approach-push-directions
           (cond ((and (eq arm :arms) (not (eq approach-arm :arms)))
                  (if (eq approach-arm :larm)
                    (list (car push-directions))
                    (cdr push-directions)))
                 (t push-directions)))
         (approach-approach-directions
           (cond ((and (eq arm :arms) (not (eq approach-arm :arms)))
                  (if (eq approach-arm :larm)
                    (list (car approach-directions))
                    (cdr approach-directions)))
                 (t approach-directions)))
         (pushed)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords)))
    (setq approached
          (push-motion-step
            approach-arm
            :motion-type :approach
            :push-axes approach-push-axes
            :push-positions approach-push-positions
            :push-directions approach-push-directions
            :approach-directions approach-approach-directions
            :hold-type hold-type
            :move-robot nil
            :tm approach-tm
            :use-torso use-torso
            :cube-pos cube-pos
            :cube-rpy cube-rpy
            :cube-height cube-height
            :cube-width cube-width
            :cube-depth cube-depth
            :robot robot :ri ri :cube cube :vw vw))
    (when approached
      (setq pushed
            (push-motion-step
              arm
              :motion-type :push
              :push-axes push-axes
              :push-positions push-positions
              :push-directions push-directions
              :approach-directions approach-directions
              :hold-type hold-type
              :move-robot nil
              :tm push-tm
              :use-torso use-torso
              :cube-pos cube-pos
              :cube-rpy cube-rpy
              :cube-height cube-height
              :cube-width cube-width
              :cube-depth cube-depth
              :robot robot :ri ri :cube cube :vw vw)))
    (if (and approached pushed)
      (when move-robot
        (add-cubes-collision-object :cubes cubes :robot robot :ri ri)
        (send ri :angle-vector
              (car (car approached)) (car (cadr approached))
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation)
        (delete-cubes-collision-object :cubes cubes :ri ri)
        (send ri :angle-vector-raw
              (car (car pushed)) (car (cadr pushed))
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (when (and approached pushed)
      (contact-cube arm :cube cube :robot robot)
      (list (append (car approached) (car pushed))
            (append (cadr approached) (cadr pushed))))))


(defun get-grasp-target-coords
  (arm &key (motion-type :grasp)
            (grasp-direction :top)
            (grasp-type :tight)
            (cube-pos nil)
            (cube-rpy nil)
            (cube-height nil)
            (cube-width nil)
            (cube-depth nil)
            (cube *box0*))
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-rpy)
    (setq cube-rpy
          (apply #'float-vector
                 (car (rpy-angle (send (send cube :copy-worldcoords) :worldrot))))))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq arm :larm) (eq arm :rarm))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from get-grasp-target-coords nil))
  (let ((target-coords
          (make-coords :pos (copy-seq cube-pos) :rpy (copy-seq cube-rpy)))
        (grasp-offset (cond ((equal grasp-type :tight) 50)
                            ((equal grasp-type :normal) 0)
                            ((equal grasp-type :light) -50)
                            ((numberp grasp-type) grasp-type)
                            (t nil))))
    ;; TODO: implement more grasp motion
    (cond
      ((equal grasp-direction :top)
       (let ((x-offset 0)
             (y-offset 0)
             (z-offset
               (+ (/ cube-height 2.0)
                  (if (eq motion-type :approach) 100 (* -1 grasp-offset)))))
         (send target-coords :translate (float-vector x-offset y-offset z-offset) :world)
         (send target-coords :rotate pi/2 :y)
         target-coords))
      (t (return-from get-grasp-target-coords nil)))
    ))


(defun grasp-motion
  (arm &rest args
       &key (grasp-direction :top)
            (grasp-type :tight)
            (move-robot t)
            (approach-tm 3000)
            (grasp-tm 3000)
            (use-torso 0.001)
            (cube-pos nil)
            (cube-rpy nil)
            (cube-height nil)
            (cube-width nil)
            (cube-depth nil)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (cubes *boxes*)
            (vw *irtviewer*)
            &allow-other-keys)
  (if (not cube-pos)
    (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-rpy)
    (setq cube-rpy
          (apply #'float-vector
                 (car (rpy-angle (send (send cube :copy-worldcoords) :worldrot))))))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq arm :larm) (eq arm :rarm))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from grasp-motion nil))
  (let* ((prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (approach-av)
         (approach-target-coords
           (get-grasp-target-coords
             arm
             :motion-type :approach
             :grasp-direction grasp-direction
             :grasp-type grasp-type
             :cube-pos cube-pos
             :cube-rpy cube-rpy
             :cube-height cube-height
             :cube-width cube-width
             :cube-depth cube-depth
             :cube cube))
         (grasp-av)
         (grasp-target-coords
           (get-grasp-target-coords
             arm
             :motion-type :grasp
             :grasp-direction grasp-direction
             :grasp-type grasp-type
             :cube-pos cube-pos
             :cube-rpy cube-rpy
             :cube-height cube-height
             :cube-width cube-width
             :cube-depth cube-depth
             :cube cube)))
    (unless (and approach-target-coords grasp-target-coords)
      (ros::ros-error "approach target coords is invalid: ~A" approach-target-coords)
      (ros::ros-error "grasp target coords is invalid   : ~A" grasp-target-coords)
      (return-from :grasp-motion nil))
    (setq approach-av
          (solve-ik-rotation-relax
            arm approach-target-coords
            :use-torso use-torso
            :ee-type :grasp
            :robot robot))
    (when approach-av
      (setq grasp-av
            (solve-ik-rotation-relax
              arm grasp-target-coords
              :use-torso use-torso
              :ee-type :grasp
              :robot robot)))
    (if (and approach-av grasp-av)
      (when move-robot
        (add-cubes-collision-object :cubes cubes :robot robot :ri ri)
        (send ri :angle-vector approach-av approach-tm
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation)
        (delete-cubes-collision-object :cubes cubes :ri ri)
        (send ri :angle-vector-raw grasp-av grasp-tm
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation)
        (start-grasp arm :ri ri :wait t)
        )
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (when (and approach-av grasp-av)
      (grasp-cube arm :cube cube :robot robot)
      (list (list approach-av grasp-av)
            (list approach-tm grasp-tm)))))


;; release motion

(defun get-release-target-coords
  (arm &key (ee-type :contact) (release-distance -100) (robot *robot*))
  (cond
    ((equal ee-type :contact)
     (get-push-release-target-coords
       arm :release-distance release-distance :robot robot))
    ((equal ee-type :grasp)
     (get-grasp-release-target-coords
       arm :release-distance release-distance :robot robot))
    (t nil)))


(defun get-push-release-target-coords (arm &key (robot *robot*) (release-distance -100))
  (let ((target-coords (send (get-contact-coords arm :robot robot) :copy-worldcoords)))
    (send target-coords :translate (float-vector 0 0 release-distance))
    target-coords))


(defun get-grasp-release-target-coords (arm &key (robot *robot*) (release-distance -100))
  (let ((target-coords (send (get-grasp-coords arm :robot robot) :copy-worldcoords)))
    (send target-coords :translate (float-vector release-distance 0 0))
    target-coords))


(defun release-and-reset-motion
  (arm &rest args &key (move-robot t) (tm 3000) (use-torso 0.001) (ee-type :contact)
                       (robot *robot*) (ri *ri*) (cube *box0*) (vw *irtviewer*)
                       (cubes *boxes*)
                       &allow-other-keys)
  (let ((avs)
        (released)
        (reseted)
        (prev-av (send robot :angle-vector)))
    (setq released
          (release-motion
            arm
            :move-robot nil :tm tm
            :use-torso use-torso
            :ee-type ee-type
            :robot robot :ri ri :cube cube :vw vw))
    (when released
      (setq reseted
            (reset-cube-manip-motion
              arm
              :move-robot nil :tm tm
              :use-torso use-torso
              :robot robot :ri ri :vw vw)))
    (if (and released reseted)
      (when move-robot
        (if (equal ee-type :grasp) (stop-grasp arm :ri ri :wait t))
        (send ri :angle-vector-raw
              (car (car released)) (car (cadr released))
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation)
        (add-cubes-collision-object :cubes cubes :robot robot :ri ri)
        (send ri :angle-vector
              (car (car reseted)) (car (cadr reseted))
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation)
        (delete-cubes-collision-object :cubes cubes :ri ri))
      (progn
        (send robot :angle-vector prev-av)))
    (send vw :draw-objects)
    (when (and released reseted)
      (list (append (car released) (car reseted))
            (append (cadr released) (cadr reseted))))))


(defun push-release-and-reset-motion (arm &rest args)
  (apply #'release-and-reset-motion (append (list arm :ee-type :contact) args)))


(defun grasp-release-and-reset-motion (arm &rest args)
  (apply #'release-and-reset-motion (append (list arm :ee-type :grasp) args)))


(defun release-motion
  (arm &rest args &key (move-robot t) (tm 3000) (use-torso 0.001) (ee-type :contact)
                       (robot *robot*) (ri *ri*) (cube *box0*) (vw *irtviewer*)
                       &allow-other-keys)
  (unless (or (eq arm :larm) (eq arm :rarm) (eq arm :arms))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from release-motion nil))
  (let ((av)
        (release-distance-max -160)
        (release-distance-step -20))
    (dotimes (release-distance-index
               (round (/ release-distance-max release-distance-step)))
      (let ((release-distance
              (- release-distance-max
                 (* release-distance-index release-distance-step))))
        (if (eq arm :arms)
          (let ((larm-target-coords
                  (get-release-target-coords
                    :larm
                    :release-distance release-distance
                    :ee-type ee-type
                    :robot robot))
                (rarm-target-coords
                  (get-release-target-coords
                    :rarm
                    :release-distance release-distance
                    :ee-type ee-type
                    :robot robot)))
            (setq av
                  (solve-ik-rotation-relax
                    :arms (list larm-target-coords rarm-target-coords)
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot)))
          (let ((target-coords
                  (get-release-target-coords
                    arm
                    :release-distance release-distance
                    :ee-type ee-type
                    :robot robot)))
            (setq av
                  (solve-ik-rotation-relax
                    arm target-coords
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot)))))
      (if av (return)))
    (when (and av move-robot)
      (send ri :angle-vector av tm
            (get-arm-controller :arms :use-torso use-torso) 0)
      (send ri :wait-interpolation))
    (send vw :draw-objects)
    (when av
      (cond
        ((equal ee-type :contact) (discontact-cube arm :cube cube))
        ((equal ee-type :grasp) (ungrasp-cube arm :cube cube))
        (t nil))
      (list (list av) (list tm)))))


(defun push-release-motion (arm &rest args)
  (apply #'release-motion (append (list arm :ee-type :contact) args)))


(defun grasp-release-motion (arm &rest args)
  (apply #'release-motion (append (list arm :ee-type :grasp) args)))


;; push hold motion

(defun push-hold-motion
  (&rest args
         &key (push-axis :y)
              (push-position :center)
              (approach-direction :back)
              (top-arm :larm)
              (front-arm nil)
              (approach-arm nil)
              (hold-type :tight)
              (move-robot t)
              (use-torso 0.001)
              (cube-pos nil)
              (cube-rpy (float-vector 0 0 0))
              (cube-height nil)
              (cube-width nil)
              (cube-depth nil)
              (robot *robot*)
              (ri *ri*)
              (cube *box0*)
              (vw *irtviewer*)
              &allow-other-keys)
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq top-arm :larm) (eq top-arm :rarm) (null top-arm))
    (ros::ros-error (format nil "top-arm is invalid: ~A" top-arm))
    (return-from push-hold-motion nil))
  (unless (or (eq front-arm :larm) (eq front-arm :rarm) (null front-arm))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (return-from push-hold-motion nil))
  (unless (or (eq approach-direction :back)
              (eq approach-direction :top)
              (eq approach-direction :side))
    (ros::ros-error (format nil "approach-direction is invalid: ~A" approach-direction))
    (return-from push-hold-motion nil))
  (when (and top-arm front-arm (eq top-arm front-arm))
    (ros::ros-error (format nil "top-arm is invalid: ~A" top-arm))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (return-from push-hold-motion nil))
  (let ((push-axes (list push-axis push-axis))
        (push-positions (list push-position push-position))
        (push-directions
          (cond ((eq push-axis :x)
                 (if (eq front-arm :larm) (list :front :back) (list :back :front)))
                ((eq push-axis :y) (list :left :right))
                ((eq push-axis :z)
                 (if (eq top-arm :larm) (list :top :bottom) (list :top :bottom)))))
        (approach-directions
          (cond ((eq approach-direction :side) (list :left :right))
                (t (list approach-direction approach-direction))))
        push-holded)
    (setq push-holded
          (push-motion
            :arms
            :push-axes push-axes
            :push-positions push-positions
            :push-directions push-directions
            :approach-directions approach-directions
            :approach-arm approach-arm
            :hold-type hold-type
            :move-robot move-robot
            :use-torso use-torso
            :cube-pos cube-pos
            :cube-rpy cube-rpy
            :cube-height cube-height
            :cube-width cube-width
            :cube-depth cube-depth
            :robot robot :ri ri :cube cube :vw vw))
    push-holded))


;; push-move-motion

(defun push-move-motion
  (arm &rest args
       &key (push-axis :y)
            (move-distance 200.0)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (vw *irtviewer*)
            &allow-other-keys)
  (let ((av)
        (prev-av (send robot :angle-vector))
        (prev-cube-coords (send cube :copy-worldcoords))
        (cube-move-vector
          (cond ((eq push-axis :x) (float-vector move-distance 0 0))
                ((eq push-axis :y) (float-vector 0 move-distance 0))
                ((eq push-axis :z) (float-vector 0 0 move-distance))
                (t (float-vector 0 0 0))))
        (target-coords nil))
    (send cube :translate cube-move-vector :world)
    (setq target-coords
          (send (get-contact-cube-coords arm :cube cube) :copy-worldcoords))
    (setq av
          (solve-ik-rotation-relax
            arm target-coords
            :use-torso use-torso :robot robot))
    (if av
      (when move-robot
        (send ri :angle-vector av tm
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list (list av) (list tm)))))


(defun push-next-to-motion
  (arm &rest args
       &key (push-axis :y)
            (cubes *boxes*)
            (cube-index 0)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (vw *irtviewer*)
            &allow-other-keys)
  (let* ((cube-coords
           (send (send cube :worldcoords) :copy-worldcoords))
         (cube-pos (send cube-coords :worldpos))
         (target-cube (elt cubes cube-index))
         (target-cube-coords
           (send (send target-cube :worldcoords) :copy-worldcoords))
         (target-cube-pos (send target-cube-coords :worldpos))
         (move-distance
           (cond
             ((equal push-axis :x)
              (- (elt target-cube-pos 0) (elt cube-pos 0)))
             ((equal push-axis :y)
              (- (elt target-cube-pos 1) (elt cube-pos 1)))
             (t nil)))
         (cube-offset
           (cond
             ((equal push-axis :x)
              (+ (/ (get-cube-depth :cube cube) 2.0)
                 (/ (get-cube-depth :cube target-cube) 2.0)))
             ((equal push-axis :y)
              (+ (/ (get-cube-width :cube cube) 2.0)
                 (/ (get-cube-width :cube target-cube) 2.0)))
             (t nil))))
    (if (and move-distance
             (> (abs move-distance) cube-offset)
             (or (and (equal push-axis :x) (equal arm :larm) (>= move-distance 0))
                 (and (equal push-axis :x) (equal arm :rarm) (>= 0 move-distance))
                 (and (equal push-axis :y) (equal arm :larm) (>= 0 move-distance))
                 (and (equal push-axis :y) (equal arm :rarm) (>= move-distance 0))))
      (progn
        (setq move-distance
              (- move-distance (* (if (> move-distance 0) 1 -1) cube-offset)))
        (push-move-motion
          arm :push-axis push-axis
          :move-distance move-distance
          :move-robot move-robot :tm tm :use-torso use-torso
          :robot robot :ri ri :cube cube :vw vw)))))


(defun push-left-wall-next-to-motion
  (arm &rest args
       &key (wall-left-limit *wall-left-limit*)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (vw *irtviewer*)
            &allow-other-keys)
  (let* ((av)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (cube-pos (send prev-cube-coords :worldpos))
         (cube-width (get-cube-width :cube cube))
         (cube-move-vector
           (float-vector 0
                         (- wall-left-limit
                            (elt cube-pos 1)
                            (/ cube-width 2.0))
                         0))
         (target-coords nil))
    (send cube :translate cube-move-vector :world)
    (setq target-coords
          (cond
            ((equal arm :arms)
             (send-all (get-contact-cube-coords arm :cube cube) :copy-worldcoords))
            (t (send (get-contact-cube-coords arm :cube cube) :copy-worldcoords))))
    (setq av
          (solve-ik-rotation-relax
            arm target-coords
            :use-torso use-torso :robot robot))
    (if av
      (when move-robot
        (send ri :angle-vector av tm
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list (list av) (list tm)))))


(defun push-front-wall-next-to-motion
  (arm &rest args
       &key (wall-front-limit *wall-front-limit*)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001)
            (robot *robot*)
            (ri *ri*)
            (cube *box0*)
            (vw *irtviewer*)
            &allow-other-keys)
  (let* ((av)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (cube-pos (send prev-cube-coords :worldpos))
         (cube-depth (get-cube-depth :cube cube))
         (cube-move-vector
           (float-vector
             (- wall-front-limit
                (elt cube-pos 0)
                (/ cube-depth 2.0))
             0
             0))
         (target-coords nil))
    (send cube :translate cube-move-vector :world)
    (setq target-coords
          (cond
            ((equal arm :arms)
             (send-all (get-contact-cube-coords arm :cube cube) :copy-worldcoords))
            (t (send (get-contact-cube-coords arm :cube cube) :copy-worldcoords))))
    (setq av
          (solve-ik-rotation-relax
            arm target-coords
            :use-torso use-torso :robot robot))
    (if av
      (when move-robot
        (send ri :angle-vector av tm
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list (list av) (list tm)))))


;; support hold motion

(defun support-hold-motion
  (&rest args
         &key (support-axis :x)
              (push-position :center)
              (approach-direction :back)
              (top-arm :rarm)
              (front-arm nil)
              (back-arm nil)
              (approach-arm nil)
              (hold-type :tight)
              (move-robot t)
              (switch nil)
              (use-torso 0.001)
              (cube-pos nil)
              (cube-rpy (float-vector 0 0 0))
              (cube-height nil)
              (cube-width nil)
              (cube-depth nil)
              (robot *robot*)
              (ri *ri*)
              (cube *box0*)
              (vw *irtviewer*)
              &allow-other-keys)
  (if (not cube-pos) (setq cube-pos (send (send cube :copy-worldcoords) :worldpos)))
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq top-arm :larm) (eq top-arm :rarm))
    (ros::ros-error (format nil "top-arm is invalid: ~A" top-arm))
    (return-from support-hold-motion nil))
  (unless (or (eq front-arm :larm) (eq front-arm :rarm) (null front-arm))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (return-from support-hold-motion nil))
  (unless (or (eq back-arm :larm) (eq back-arm :rarm) (null back-arm))
    (ros::ros-error (format nil "back-arm is invalid: ~A" back-arm))
    (return-from support-hold-motion nil))
  (unless (or (and (eq support-axis :x) (eq approach-direction :back))
              (and (eq support-axis :y) (eq approach-direction :side)))
    (ros::ros-error (format nil "support-axis is invalid: ~A" support-axis))
    (ros::ros-error (format nil "approach-direction is invalid: ~A" approach-direction))
    (return-from support-hold-motion nil))
  (when (or (eq top-arm front-arm)
            (eq top-arm back-arm))
    (ros::ros-error (format nil "top-arm is invalid: ~A" top-arm))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (ros::ros-error (format nil "back-arm is invalid: ~A" back-arm))
    (return-from support-hold-motion nil))
  (let* ((push-axes
          (cond ((eq support-axis :x)
                 (if (eq top-arm :larm) (list :z :y) (list :y :z)))
                ((eq support-axis :y)
                 (if (eq top-arm :larm) (list :z :x) (list :x :z)))
                (t nil)))
         (push-directions
           (cond ((eq support-axis :x)
                  (if (eq top-arm :larm) (list :top :right) (list :left :top)))
                 ((eq support-axis :y)
                  (if (eq top-arm :larm)
                    (list :top
                          (cond ((and (eq front-arm :rarm) (null back-arm)) :front)
                                ((and (null front-arm) (eq back-arm :rarm)) :back)
                                (t nil)))
                    (list (cond ((and (eq front-arm :larm) (null back-arm)) :front)
                                ((and (null front-arm) (eq back-arm :larm)) :back)
                                (t nil))
                          :top)))
                 (t nil)))
         (push-positions (list push-position push-position))
         (approach-directions
           (cond ((and (eq support-axis :x) (eq approach-direction :back))
                  (if switch
                    (list :back :back)
                    (if (or (eq (car push-directions) :top)
                            (eq (car push-directions) :bottom))
                      (list :left :back) (list :back :right))))
                 ((and (eq support-axis :y) (eq approach-direction :side))
                  (if switch
                    (list :left :right)
                    (if (or (eq (car push-directions) :top)
                            (eq (car push-directions) :bottom))
                      (list :left :top) (list :top :right))))
                 (t (list approach-direction approach-direction))))
         support-holded)
    (setq support-holded
          (push-motion
            :arms
            :push-axes push-axes
            :push-positions push-positions
            :push-directions push-directions
            :approach-directions approach-directions
            :approach-arm approach-arm
            :hold-type hold-type
            :move-robot move-robot
            :use-torso use-torso
            :cube-pos cube-pos
            :cube-rpy cube-rpy
            :cube-height cube-height
            :cube-width cube-width
            :cube-depth cube-depth
            :robot robot :ri ri :cube cube :vw vw))
    support-holded))


;; hold release motion

(defun hold-release-motion (arm &rest args
                                &key (tm 5000)
                                     &allow-other-keys)
  (apply #'push-release-motion (append (list arm :tm tm) args)))


;; hold lift up & down motion

(defun lift-up-motion
  (arm &rest args &key (lift-height 100) (move-robot t) (ee-type :contact)
                       (tm 1000) (use-torso 0.001) (reset-rpy nil)
                       (robot *robot*) (ri *ri*) (cube *box0*) (vw *irtviewer*)
                       &allow-other-keys)
  (let* ((av)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (height-step 50)
         (height-count 0)
         (max-height-count (+ (floor (/ (abs lift-height) height-step)) 1))
         (avs nil)
         (tms nil))
    (setq height-step (/ lift-height max-height-count))
    ;; lift up
    (block :lift-up-motion-step
      (while (> max-height-count height-count)
        (send cube :translate (float-vector 0 0 height-step) :world)
        (if reset-rpy
          (send cube :newcoords
                (make-coords :pos (copy-seq (send cube :worldpos))
                             :rpy (float-vector 0 0 0))))
        (if (equal arm :arms)
          (let ((larm-target-coords
                  (send (get-ee-cube-coords
                          :larm :cube cube :ee-type ee-type)
                        :copy-worldcoords))
                (rarm-target-coords
                  (send (get-ee-cube-coords
                          :rarm :cube cube :ee-type ee-type)
                        :copy-worldcoords)))
            (setq av
                  (solve-ik-rotation-relax
                    :arms (list larm-target-coords rarm-target-coords)
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot)))
          (let ((target-coords
                  (send (get-ee-cube-coords
                          arm :cube cube :ee-type ee-type)
                        :copy-worldcoords)))
            (setq av
                  (solve-ik-rotation-relax
                    arm target-coords
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot))))
        (send vw :draw-objects)
        (if av
          (progn
            (setq avs (append avs (list av)))
            (setq tms (append tms (list tm))))
          (return-from :lift-up-motion-step nil))
        (setq height-count (+ height-count 1))))
    (if av
      (when move-robot
        (send ri :angle-vector-sequence avs tms
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list avs tms))))


(defun hold-lift-up-motion (&rest args)
  (apply #'lift-up-motion (append (list :arms :ee-type :contact) args)))


(defun grasp-lift-up-motion (arm &rest args)
  (apply #'lift-up-motion (append (list arm :ee-type :grasp) args)))


(defun lift-down-motion
  (arm &rest args &key (lift-height nil) (cube-pos-x nil) (cube-pos-y nil)
                       (move-robot t) (ee-type :contact)
                       (tm 500) (use-torso 0.001)
                       (robot *robot*) (ri *ri*) (cube *box0*)
                       (vw *irtviewer*)
                       (table-z *table-z*) (table-height *table-height*)
                       &allow-other-keys)
  (unless cube-pos-x
    (setq cube-pos-x (elt (send cube :get :initial-pos) 0)))
  (unless cube-pos-y
    (setq cube-pos-y (elt (send cube :get :initial-pos) 1)))
  (let* ((av)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (cube-pos (send prev-cube-coords :worldpos))
         (cube-height (get-cube-height :cube cube))
         (height-step 50)
         (height-count 0)
         (max-height-count)
         (execute-move
           (or (> (round (abs (- cube-pos-x (elt cube-pos 0)))) 0)
               (> (round (abs (- cube-pos-y (elt cube-pos 1)))) 0)))
         (moved nil)
         (avs nil)
         (tms nil))
    (unless lift-height
      (setq lift-height (- (- (elt cube-pos 2) (/ cube-height 2.0)) (+ table-z table-height))))
    (setq max-height-count (+ (floor (/ (abs lift-height) height-step)) 1))
    (setq height-step (/ (* -1.0 lift-height) max-height-count))
    (if execute-move
      (setq moved
            (lift-move-motion
              arm :ee-type ee-type
              :cube-pos-x cube-pos-x
              :cube-pos-y cube-pos-y
              :cube-pos-z (elt cube-pos 2)
              :move-robot nil :tm tm :use-torso use-torso
              :robot robot :ri ri :cube cube :vw vw)))
    ;; lift down
    (if (or (null execute-move)
            (and execute-move moved))
      (block :lift-down-motion-step
        (while (> max-height-count height-count)
          (send cube :translate (float-vector 0 0 height-step) :world)
          (if (equal arm :arms)
            (let ((larm-target-coords
                    (send (get-ee-cube-coords
                            :larm :cube cube :ee-type ee-type)
                          :copy-worldcoords))
                  (rarm-target-coords
                    (send (get-ee-cube-coords
                            :rarm :cube cube :ee-type ee-type)
                          :copy-worldcoords)))
              (setq av
                    (solve-ik-rotation-relax
                      :arms (list larm-target-coords rarm-target-coords)
                      :use-torso use-torso
                      :ee-type ee-type
                      :robot robot)))
            (let ((target-coords
                    (send (get-ee-cube-coords
                            arm :cube cube :ee-type ee-type)
                          :copy-worldcoords)))
              (setq av
                    (solve-ik-rotation-relax
                      arm target-coords
                      :use-torso use-torso
                      :ee-type ee-type
                      :robot robot))))
          (send vw :draw-objects)
          (if av
            (progn
              (setq avs (append avs (list av)))
              (setq tms (append tms (list tm))))
            (return-from :lift-down-motion-step nil))
          (setq height-count (+ height-count 1)))))
    (if av
      (when move-robot
        (if execute-move
          (progn
            (send ri :angle-vector-sequence-raw
                  (car moved) (cadr moved)
                  (get-arm-controller :arms :use-torso use-torso) 0)
            (send ri :wait-interpolation)))
        (send ri :angle-vector-sequence-raw avs tms
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av
      (if execute-move
        (list (append (car moved) avs) (append (cadr moved) tms))
        (list avs tms)))))


(defun hold-lift-down-motion (&rest args)
  (apply #'lift-down-motion (append (list :arms :ee-type :contact) args)))


(defun grasp-lift-down-motion (arm &rest args)
  (apply #'lift-down-motion (append (list arm :ee-type :grasp) args)))


(defun lift-move-motion
  (arm &rest args &key (cube-pos-x nil) (cube-pos-y nil) (cube-pos-z nil)
                       (move-robot t) (ee-type :contact)
                       (tm 1000) (use-torso 0.001)
                       (robot *robot*) (ri *ri*) (cube *box0*) (vw *irtviewer*)
                       &allow-other-keys)
  (unless cube-pos-x
    (setq cube-pos-x (elt (send cube :get :initial-pos) 0)))
  (unless cube-pos-y
    (setq cube-pos-y (elt (send cube :get :initial-pos) 1)))
  (unless cube-pos-z
    (setq cube-pos-z (elt (send cube :get :initial-pos) 2)))
  (let* ((av)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (cube-pos (send prev-cube-coords :worldpos))
         (move-step 50)
         (move-count 0)
         (cube-pos-x-diff (- cube-pos-x (elt cube-pos 0)))
         (cube-pos-y-diff (- cube-pos-y (elt cube-pos 1)))
         (cube-pos-z-diff (- cube-pos-z (elt cube-pos 2)))
         (max-move-count
           (+ (max
                (floor (/ (abs cube-pos-x-diff) move-step))
                (floor (/ (abs cube-pos-y-diff) move-step))
                (floor (/ (abs cube-pos-z-diff) move-step)))
              1))
         (cube-pos-x-step (/ cube-pos-x-diff max-move-count))
         (cube-pos-y-step (/ cube-pos-y-diff max-move-count))
         (cube-pos-z-step (/ cube-pos-z-diff max-move-count))
         (avs nil)
         (tms nil))
    (block :lift-move-motion-step
      (while (> max-move-count move-count)
        (send cube :translate
              (float-vector cube-pos-x-step cube-pos-y-step cube-pos-z-step) :world)
        (if (equal arm :arms)
          (let ((larm-target-coords
                  (send (get-ee-cube-coords
                          :larm :cube cube :ee-type ee-type)
                        :copy-worldcoords))
                (rarm-target-coords
                  (send (get-ee-cube-coords
                          :rarm :cube cube :ee-type ee-type)
                        :copy-worldcoords)))
            (setq av
                  (solve-ik-rotation-relax
                    :arms (list larm-target-coords rarm-target-coords)
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot)))
          (let ((target-coords
                  (send (get-ee-cube-coords
                          arm :cube cube :ee-type ee-type)
                        :copy-worldcoords)))
            (setq av
                  (solve-ik-rotation-relax
                    arm target-coords
                    :use-torso use-torso
                    :ee-type ee-type
                    :robot robot))))
        (send vw :draw-objects)
        (if av
          (progn
            (setq avs (append avs (list av)))
            (setq tms (append tms (list tm))))
          (return-from :lift-move-motion-step nil))
        (setq move-count (+ move-count 1))))
    (if av
      (when move-robot
        (send ri :angle-vector-sequence avs tms
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list avs tms))))


(defun hold-lift-move-motion (&rest args)
  (apply #'lift-move-motion (append (list :arms :ee-type :contact) args)))


(defun grasp-lift-move-motion (arm &rest args)
  (apply #'lift-move-motion (append (list arm :ee-type :grasp) args)))


(defun pile-up-motion
  (arm &rest args &key (cubes *boxes*) (cube-index 0)
                       (move-robot t) (ee-type :contact)
                       (tm 1000) (use-torso 0.001)
                       (robot *robot*) (ri *ri*) (cube *box0*)
                       (vw *irtviewer*)
                       &allow-other-keys)
  (let* ((avs)
         (lifted-up)
         (moved)
         (lifted-down)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (cube-height (get-cube-height :cube cube))
         (cube-coords (send (send cube :worldcoords) :copy-worldcoords))
         (cube-pos (send cube-coords :worldpos))
         (target-cube (elt cubes cube-index))
         (target-cube-height (get-cube-height :cube target-cube))
         (target-cube-coords
           (send (send target-cube :worldcoords) :copy-worldcoords))
         (target-cube-pos (send target-cube-coords :worldpos))
         (target-cube-top-z
           (+ (elt target-cube-pos 2) (/ target-cube-height 2.0)))
         (lift-height nil)
         (lift-height-offset 100))
    (setq lifted-up
          (lift-up-motion
            arm :ee-type ee-type
            :lift-height (- (+ target-cube-top-z lift-height-offset)
                            (- (elt cube-pos 2) (/ cube-height 2.0)))
            :move-robot nil :tm tm :use-torso use-torso
            :robot robot :ri ri :cube cube :vw vw))
    (setq cube-pos (send (send (send cube :worldcoords) :copy-worldcoords) :worldpos))
    (setq lift-height
          (- (- (elt cube-pos 2) (/ cube-height 2.0)) target-cube-top-z))
    (when lifted-up
      (setq moved
            (lift-move-motion
              arm :ee-type ee-type
              :cube-pos-x (elt target-cube-pos 0)
              :cube-pos-y (elt target-cube-pos 1)
              :cube-pos-z (elt cube-pos 2)
              :move-robot nil :tm tm :use-torso use-torso
              :robot robot :ri ri :cube cube :vw vw)))
    (when moved
      (setq lifted-down
            (lift-down-motion
              arm :ee-type ee-type
              :lift-height lift-height
              :cube-pos-x (elt target-cube-pos 0)
              :cube-pos-y (elt target-cube-pos 1)
              :move-robot nil :tm tm :use-torso use-torso
              :robot robot :ri :ri :cube cube :vw vw)))
    (if (and lifted-up moved lifted-down)
      (when move-robot
        (send ri :angle-vector-sequence
              (append (car lifted-up) (car moved) (car lifted-down))
              (append (cadr lifted-up) (cadr moved) (cadr lifted-down))
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when (and lifted-up moved lifted-down)
      (list (append (car lifted-up) (car moved) (car lifted-down))
            (append (cadr lifted-up) (cadr moved) (cadr lifted-down))))))


(defun hold-pile-up-motion (&rest args)
  (apply #'pile-up-motion (append (list :arms :ee-type :contact) args)))


(defun grasp-pile-up-motion (arm &rest args)
  (apply #'pile-up-motion (append (list arm :ee-type :grasp) args)))


;; rotate motion

(defun get-rotate-distance
  (&key (current-angle) (cube-norm-length) (initial-angle 0) (rotate-axis :x))
  (unless (or (eq rotate-axis :x) (eq rotate-axis :y) (eq rotate-axis :z))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from get-rotate-distance nil))
  (* (sin (abs (+ initial-angle current-angle)))
     (/ cube-norm-length 2.0)))


(defun rotate-motion
  (&rest args
         &key (rotate-angle pi/2)
              (rotate-axis :x)
              (rotate-type :push)
              (initial-rotate-angle 0)
              (move-robot t)
              (tm 1000)
              (use-torso 0.001)
              (cube-height nil)
              (cube-depth nil)
              (cube-width nil)
              (use-rotate-distance t)
              (robot *robot*)
              (ri *ri*)
              (cube *box0*)
              (vw *irtviewer*)
              &allow-other-keys)
  (if (not cube-height) (setq cube-height (get-cube-height :cube cube)))
  (if (not cube-width) (setq cube-width (get-cube-width :cube cube)))
  (if (not cube-depth) (setq cube-depth (get-cube-depth :cube cube)))
  (unless (or (eq rotate-axis :x) (eq rotate-axis :y) (eq rotate-axis :z))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from rotate-motion nil))
  (unless (and (send cube :get :larm-contact-coords)
               (send cube :get :rarm-contact-coords))
    (ros::ros-error "robot is not contacted to cube")
    (return-from rotate-motion nil))
  (let* ((av)
         (rotate-distance)
         (prev-rotate-distance)
         (rotate-step (if (> rotate-angle 0) 0.1 -0.1))
         (rotate-count 0)
         (max-rotate-count (floor (/ rotate-angle rotate-step)))
         (cube-norm-length
           (cond ((eq rotate-axis :x) (norm (float-vector cube-width cube-height)))
                 ((eq rotate-axis :y) (norm (float-vector cube-depth cube-height)))
                 ((eq rotate-axis :z) (norm (float-vector cube-width cube-depth)))
                 (t nil)))
         (initial-angle
           (+ initial-rotate-angle
              (* (cond ((equal initial-rotate-angle 0)
                        (if (> rotate-angle 0) 1.0 -1.0))
                       ((> initial-rotate-angle 0) 1.0)
                       (t -1.0))
                 (cond ((eq rotate-axis :x) (atan (/ cube-height cube-width)))
                       ((eq rotate-axis :y) (atan (/ cube-height cube-depth)))
                       ((eq rotate-axis :z) (atan (/ cube-depth cube-width)))
                       (t 0)))))
         (current-angle 0)
         (prev-av (send robot :angle-vector))
         (prev-cube-coords (send cube :copy-worldcoords))
         (rotate-avs nil)
         (rotate-tms nil)
         (larm-target-coords nil)
         (rarm-target-coords nil))
    (if use-rotate-distance
      (setq prev-rotate-distance
            (get-rotate-distance
              :current-angle current-angle
              :cube-norm-length cube-norm-length
              :initial-angle initial-angle
              :rotate-axis rotate-axis)))
    (block :rotate-motion-step
      (while (> max-rotate-count rotate-count)
        (setq current-angle (+ current-angle rotate-step))
        (send cube :rotate rotate-step rotate-axis :world)
        (if use-rotate-distance
          (progn
            (setq rotate-distance
                  (get-rotate-distance
                    :current-angle current-angle
                    :cube-norm-length cube-norm-length
                    :initial-angle initial-angle
                    :rotate-axis rotate-axis))
            (cond
              ((or (eq rotate-axis :x)(eq rotate-axis :y))
               (send cube :translate
                     (float-vector 0 0 (- rotate-distance prev-rotate-distance)) :world))
              ((eq rotate-axis :z)
               (send cube :translate
                     (float-vector (- rotate-distance prev-rotate-distance) 0 0) :world))
              (t nil))
            (setq prev-rotate-distance rotate-distance)))
        (setq larm-target-coords
              (send (get-contact-cube-coords :larm :cube cube) :copy-worldcoords))
        (setq rarm-target-coords
              (send (get-contact-cube-coords :rarm :cube cube) :copy-worldcoords))
        (setq av
              (solve-ik-rotation-relax
                :arms (list larm-target-coords rarm-target-coords)
                :use-torso use-torso :robot robot))
        (send vw :draw-objects)
        (if av
          (progn
            (setq rotate-avs (append rotate-avs (list av)))
            (setq rotate-tms (append rotate-tms (list tm))))
          (return-from :rotate-motion-step nil))
        (setq rotate-count (+ rotate-count 1)))
      (send cube :rotate (- rotate-angle current-angle) rotate-axis :world)
      (setq current-angle rotate-angle)
      (if use-rotate-distance
        (progn
          (setq rotate-distance
                (get-rotate-distance
                  :current-angle current-angle
                  :cube-norm-length cube-norm-length
                  :initial-angle initial-angle
                  :rotate-axis rotate-axis))
          (cond
            ((or (eq rotate-axis :x) (eq rotate-axis :y))
             (send cube :translate
                   (float-vector 0 0 (- rotate-distance prev-rotate-distance)) :world))
            ((eq rotate-axis :z)
             (send cube :translate
                   (float-vector (- rotate-distance prev-rotate-distance) 0 0) :world))
            (t nil))))
      (setq larm-target-coords
            (send (get-contact-cube-coords :larm :cube cube) :copy-worldcoords))
      (setq rarm-target-coords
            (send (get-contact-cube-coords :rarm :cube cube) :copy-worldcoords))
      (setq av
            (solve-ik-rotation-relax
              :arms (list larm-target-coords rarm-target-coords)
              :use-torso use-torso :robot robot))
      (send vw :draw-objects)
      (if av
        (progn
          (setq rotate-avs (append rotate-avs (list av)))
          (setq rotate-tms (append rotate-tms (list tm))))
        (return-from :rotate-motion-step nil)))
    (if av
      (when move-robot
        ;; raw usage
        (send ri :angle-vector-sequence-raw rotate-avs rotate-tms
              (get-arm-controller :arms :use-torso use-torso) 0)
        (send ri :wait-interpolation))
      (progn
        (send robot :angle-vector prev-av)
        (send cube :newcoords prev-cube-coords)
        (send cube :worldcoords)))
    (send vw :draw-objects)
    (when av (list rotate-avs rotate-tms))))


(defun push-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :push) args)))


(defun support-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :support) args)))


;; switch rotate motion

(defun get-switch-rotate-angle (&key (rotate-angle pi/2) (rotate-axis :x) (offset 0.1)
                                     (cube *box0*))
  (let ((cube-height (get-cube-height :cube cube))
        (cube-width (get-cube-width :cube cube))
        (cube-depth (get-cube-depth :cube cube)))
    (* (if (> rotate-angle 0) 1.0 -1.0)
       (- (cond ((eq rotate-axis :x)
                 (- pi/2 (atan (/ cube-height cube-width))))
                ((eq rotate-axis :y)
                 (- pi/2 (atan (/ cube-height cube-depth))))
                ((eq rotate-axis :z) (/ pi 4)))
          offset))))


;; reset-motion

(defun reset-motion (&rest args
                           &key (move-robot t) (tm 5000) (use-torso 0.001)
                                (robot *robot*) (ri *ri*)
                                (vw *irtviewer*)
                                &allow-other-keys)
  (let ((av (send robot :reset-cube-manip-pose)))
    (send vw :draw-objects)
    (when move-robot
      (stop-grasp :arms :ri ri :wait t)
      ;: (start-grasp :arms :ri ri)
      (send ri :angle-vector av tm
            (get-arm-controller :arms :use-torso use-torso) 0)
      (send ri :wait-interpolation))
    (list (list av) (list tm))))


(defun reset-cube-manip-motion (arm &rest args
                                    &key (move-robot t) (tm 5000) (use-torso 0.001)
                                         (robot *robot*) (ri *ri*) (vw *irtviewer*)
                                         &allow-other-keys)
  (let ((av (send robot :reset-cube-manip-pose arm)))
    (when move-robot
      (send ri :angle-vector av tm
            (get-arm-controller arm :use-torso use-torso) 0)
      (send ri :wait-interpolation))
    (send vw :draw-objects)
    (list (list av) (list tm))))


(provide :common-motions "common-motions.l")
