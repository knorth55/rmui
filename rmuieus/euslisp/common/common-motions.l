#!/usr/bin/env roseus

;; (require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")


;; init

(defun motion-init (&key (cube-pos-y 0))
  (scene-init :cube-pos-y cube-pos-y)
  (add-cube-coords)
  (reset-motion))


(defun contact-cube (arm)
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (contact-cube-step a)))


(defun contact-cube-step (arm)
  (let* ((coords-name (if (eq arm :larm) :larm-contact-coords :rarm-contact-coords))
         (contact-coords
           (make-cascoords :name coords-name
                           :coords (send (send *robot* arm :end-coords) :copy-worldcoords))))
    (send *cube* :assoc contact-coords)
    (send *cube* :put coords-name contact-coords)))


(defun discontact-cube (arm)
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (discontact-cube-step a)))


(defun discontact-cube-step (arm)
  (let ((coords-name (if (eq arm :larm) :larm-contact-coords :rarm-contact-coords)))
    (send *cube* :dissoc (send *cube* :get coords-name))
    (send *cube* :put coords-name nil)))


(defun assoc-cube (arm)
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (assoc-cube-step a)))


(defun assoc-cube-step (arm)
  (if (send *ri* :simulation-modep)
    (send *ri* :robot arm :end-coords :assoc (car (send *ri* :find-object *cube-id*)))))


(defun dissoc-cube (arm)
  (dolist (a (if (eq arm :arms) (list :larm :rarm) (list arm)))
    (dissoc-cube-step a)))


(defun dissoc-cube-step (arm)
  (if (send *ri* :simulation-modep)
    (send *ri* :robot arm :end-coords :dissoc (car (send *ri* :find-object *cube-id*)))))


(defun add-cube-coords ()
  (let* ((cube-depth (x-of-cube *cube*))
         (cube-width (y-of-cube *cube*))
         (cube-height (z-of-cube *cube*))
         (cube-coords (send (send *cube* :worldcoords) :copy-worldcoords))
         (front-cube-coords
           (make-cascoords :name :front-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector (/ cube-depth 2.0) 0 0) :world)))
         (back-cube-coords
           (make-cascoords :name :back-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector (/ cube-depth -2.0) 0 0) :world)))
         (left-cube-coords
           (make-cascoords :name :left-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 (/ cube-width 2.0) 0) :world)))
         (right-cube-coords
           (make-cascoords :name :right-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 (/ cube-width -2.0) 0) :world)))
         (top-cube-coords
           (make-cascoords :name :top-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 0 (/ cube-height 2.0)) :world)))
         (bottom-cube-coords
           (make-cascoords :name :bottom-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 0 (/ cube-height -2.0)) :world))))
    (send *cube* :assoc front-cube-coords)
    (send *cube* :assoc back-cube-coords)
    (send *cube* :assoc left-cube-coords)
    (send *cube* :assoc right-cube-coords)
    (send *cube* :assoc top-cube-coords)
    (send *cube* :assoc bottom-cube-coords)
    (send *cube* :put :front-cube-coords front-cube-coords)
    (send *cube* :put :back-cube-coords back-cube-coords)
    (send *cube* :put :left-cube-coords left-cube-coords)
    (send *cube* :put :right-cube-coords right-cube-coords)
    (send *cube* :put :top-cube-coords top-cube-coords)
    (send *cube* :put :bottom-cube-coords bottom-cube-coords)))


(defun get-cube-length (axis &key (cube-rpy (float-vector 0 0 0)))
  (let* ((inverse-coords (send (make-coords :rpy cube-rpy) :inverse-transformation))
         (cube-cpos (send (send *cube* :copy-worldcoords) :worldpos))
         (cube-coords
           (list (send (send *cube* :get :front-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :back-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :left-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :right-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :top-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :bottom-cube-coords) :copy-worldcoords)))
         (cube-pos-list
           (mapcar
             #'(lambda (c)
                 (elt (send inverse-coords :rotate-vector (v- (send c :worldpos) cube-cpos))
                      (cond ((eq axis :x) 0) ((eq axis :y) 1) ((eq axis :z) 2) (t nil))))
             cube-coords)))
    (- (reduce #'max cube-pos-list) (reduce #'min cube-pos-list))))


(defun get-cube-depth (&key (cube-rpy (float-vector 0 0 0)))
  (get-cube-length :x :cube-rpy cube-rpy))
(defun get-cube-width (&key (cube-rpy (float-vector 0 0 0)))
  (get-cube-length :y :cube-rpy cube-rpy))
(defun get-cube-height (&key (cube-rpy (float-vector 0 0 0)))
  (get-cube-length :z :cube-rpy cube-rpy))


(defun get-cube-coords (direction)
  (let* ((cube-coords
           (list (send (send *cube* :get :front-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :back-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :left-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :right-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :top-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :bottom-cube-coords) :copy-worldcoords)))
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


(defun get-front-cube-coords () (get-cube-coords :front))
(defun get-back-cube-coords () (get-cube-coords :back))
(defun get-left-cube-coords () (get-cube-coords :left))
(defun get-right-cube-coords () (get-cube-coords :right))
(defun get-top-cube-coords () (get-cube-coords :top))
(defun get-bottom-cube-coords () (get-cube-coords :bottom))


;; motions
;; push motion

(defun get-push-target-coords
  (arm &key (motion-type :push)
            (push-axis :y)
            (push-position :center)
            (push-direction :left)
            (approach-direction :back)
            (hold-type :tight)
            (cube-pos (send (send *cube* :copy-worldcoords) :worldpos))
            (cube-rpy (float-vector 0 0 0))
            (cube-height (get-cube-height))
            (cube-width (get-cube-width))
            (cube-depth (get-cube-depth)))
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
                  (+ (/ cube-depth 2.0) (if (eq motion-type :approach) 50 0)))))
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
                  (+ (/ cube-width 2.0) (if (eq motion-type :approach) 50 0)))))
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
                       (if (eq push-direction :top) 50 200) 0)))))
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
              (send target-coords :rotate (* (if (eq arm :rarm) 1.0 -1.0) pi/2) :z)
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
            (cube-pos (send (send *cube* :copy-worldcoords) :worldpos))
            (cube-rpy (float-vector 0 0 0))
            (cube-height (get-cube-height))
            (cube-width (get-cube-width))
            (cube-depth (get-cube-depth)))
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
                :cube-depth cube-depth))
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
                :cube-depth cube-depth)))
        (unless (and larm-target-coords rarm-target-coords)
          (ros::ros-error "larm target coords is invalid: ~A" larm-target-coords)
          (ros::ros-error "rarm target coords is invalid: ~A" rarm-target-coords)
          (return-from push-motion-step nil))
        (setq av (send *robot* :inverse-kinematics
                       (list larm-target-coords rarm-target-coords)
                       :move-target (list (send *robot* :larm :end-coords)
                                          (send *robot* :rarm :end-coords))
                       :rotation-axis (list t t)
                       :use-torso use-torso))
        (unless av
          (setq av (send *robot* :inverse-kinematics
                         (list larm-target-coords rarm-target-coords)
                         :move-target (list (send *robot* :larm :end-coords)
                                            (send *robot* :rarm :end-coords))
                         :rotation-axis (list :z :z)
                         :use-torso use-torso))))
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
                :cube-depth cube-depth)))
        (unless target-coords
          (return-from push-motion-step nil))
        (setq av (send *robot* arm :inverse-kinematics target-coords
                       :rotation-axis t
                       :use-torso use-torso))
        (unless av
          (setq av (send *robot* arm :inverse-kinematics target-coords
                         :rotation-axis :z
                         :use-torso use-torso)))))
    (when (and av move-robot)
      (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    (list av tm)))


(defun push-motion
  (arm &key (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (approach-arm nil)
            (hold-type :tight)
            (move-robot t)
            (push-tm 3000)
            (approach-tm 3000)
            (use-torso 0.001)
            (cube-pos (send (send *cube* :copy-worldcoords) :worldpos))
            (cube-rpy (float-vector 0 0 0))
            (cube-height (get-cube-height))
            (cube-width (get-cube-width))
            (cube-depth (get-cube-depth)))
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
         (pushed))
    (setq approached
          (push-motion-step
            approach-arm
            :motion-type :approach
            :push-axes approach-push-axes
            :push-positions approach-push-positions
            :push-directions approach-push-directions
            :approach-directions approach-approach-directions
            :hold-type hold-type
            :move-robot move-robot
            :tm approach-tm
            :use-torso use-torso
            :cube-pos cube-pos
            :cube-rpy cube-rpy
            :cube-height cube-height
            :cube-width cube-width
            :cube-depth cube-depth))
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
              :move-robot move-robot
              :tm push-tm
              :use-torso use-torso
              :cube-pos cube-pos
              :cube-rpy cube-rpy
              :cube-height cube-height
              :cube-width cube-width
              :cube-depth cube-depth)))
    (when pushed
      (contact-cube arm)
      (list (list (car approached) (car pushed))
            (list (cadr approached) (cadr pushed))))))


;; push release motion

(defun get-push-release-target-coords (arm)
  (let ((target-coords (send (send *robot* arm :end-coords) :copy-worldcoords)))
    (send target-coords :translate (float-vector 0 0 -50))
    target-coords))


(defun push-release-motion (arm &key (move-robot t) (tm 3000) (use-torso 0.001))
  (unless (or (eq arm :larm) (eq arm :rarm) (eq arm :arms))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from push-release-motion nil))
  (let ((av))
    (if (eq arm :arms)
      (let ((larm-target-coords (get-push-release-target-coords :larm))
            (rarm-target-coords (get-push-release-target-coords :rarm)))
        (setq av (send *robot* :inverse-kinematics
                       (list larm-target-coords rarm-target-coords)
                       :move-target (list (send *robot* :larm :end-coords)
                                          (send *robot* :rarm :end-coords))
                       :rotation-axis (list t t)
                       :use-torso use-torso))
        (unless av
          (setq av (send *robot* :inverse-kinematics
                         (list larm-target-coords rarm-target-coords)
                         :move-target (list (send *robot* :larm :end-coords)
                                            (send *robot* :rarm :end-coords))
                         :rotation-axis (list :z :z)
                         :use-torso use-torso))))
      (let ((target-coords (get-push-release-target-coords arm)))
        (setq av (send *robot* arm :inverse-kinematics target-coords
                       :rotation-axis t :use-torso use-torso))
        (unless av
          (setq av (send *robot* arm :inverse-kinematics target-coords
                         :rotation-axis :z :use-torso use-torso)))))
    (when (and av move-robot)
      (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    (when av
      (discontact-cube arm)
      (list (list av) (list tm)))))


;; push hold motion

(defun push-hold-motion
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (top-arm :larm)
        (front-arm nil)
        (approach-arm nil)
        (hold-type :tight)
        (move-robot t)
        (use-torso 0.001)
        (cube-pos (send (send *cube* :copy-worldcoords) :worldpos))
        (cube-rpy (float-vector 0 0 0))
        (cube-height (get-cube-height))
        (cube-width (get-cube-width))
        (cube-depth (get-cube-depth)))
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
            :cube-depth cube-depth))
    push-holded))


;; push-move-motion

(defun push-move-motion
  (arm &key (push-axis :y)
            (move-distance 200.0)
            (move-robot t)
            (tm 3000)
            (use-torso 0.001))
  (let ((av)
        (prev-av (send *robot* :angle-vector))
        (prev-cube-coords (send *cube* :copy-worldcoords))
        (cube-move-vector
          (cond ((eq push-axis :x) (float-vector move-distance 0 0))
                ((eq push-axis :y) (float-vector 0 move-distance 0))
                ((eq push-axis :z) (float-vector 0 0 move-distance))
                (t (float-vector 0 0 0)))))
    (send *cube* :translate cube-move-vector :world)
    (setq av (send *robot* arm :inverse-kinematics
                   (if (eq arm :larm)
                     (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                           :copy-worldcoords)
                     (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                           :copy-worldcoords))
                   :rotation-axis t
                   :move-target
                   (if (eq arm :larm)
                     (send *robot* :larm :end-coords)
                     (send *robot* :rarm :end-coords))
                   :use-torso use-torso))
    (unless av
      (setq av (send *robot* arm :inverse-kinematics
                     (if (eq arm :larm)
                       (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                             :copy-worldcoords)
                       (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                             :copy-worldcoords))
                     :rotation-axis t
                     :move-target
                     (if (eq arm :larm)
                       (send *robot* :larm :end-coords)
                       (send *robot* :rarm :end-coords))
                     :use-torso use-torso)))
    (if av
      (when move-robot
        (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
        (send *ri* :wait-interpolation))
      (progn
        (send *robot* :angle-vector prev-av)
        (send *cube* :newcoords prev-cube-coords)
        (send *cube* :worldcoords)))
    (send *irtviewer* :draw-objects)
    (when av (list (list av) (list tm)))))


;; support hold motion

(defun support-hold-motion
  (&key (support-axis :x)
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
        (cube-pos (send (send *cube* :copy-worldcoords) :worldpos))
        (cube-rpy (float-vector 0 0 0))
        (cube-height (get-cube-height))
        (cube-width (get-cube-width))
        (cube-depth (get-cube-depth)))
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
            :cube-depth cube-depth))
    support-holded))


;; hold release motion

(defun hold-release-motion (&key (arm :arms) (move-robot t) (tm 5000) (use-torso 0.001))
  (push-release-motion arm :move-robot move-robot :tm tm :use-torso use-torso))


;; hold lift up & down motion

(defun hold-lift-up-motion (&key (lift-height 100) (move-robot t) (tm 5000) (use-torso 0.001))
  (let ((av)
        (prev-av (send *robot* :angle-vector))
        (prev-cube-coords (send *cube* :copy-worldcoords)))
    ;; lift up
    (send *cube* :translate (float-vector 0 0 lift-height) :world)
    (setq av (send *robot* :inverse-kinematics
                   (list
                     (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                           :copy-worldcoords)
                     (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                           :copy-worldcoords))
                   :rotation-axis (list t t)
                   :move-target (list (send *robot* :larm :end-coords)
                                   (send *robot* :rarm :end-coords))
                   :use-torso use-torso))
    (unless av
      (setq av (send *robot* :inverse-kinematics
                     (list
                       (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                             :copy-worldcoords)
                       (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                             :copy-worldcoords))
                     :rotation-axis (list :z :z)
                     :move-target (list (send *robot* :larm :end-coords)
                                        (send *robot* :rarm :end-coords))
                     :use-torso use-torso)))
    (if av
      (when move-robot
        (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
        (send *ri* :wait-interpolation))
      (progn
        (send *robot* :angle-vector prev-av)
        (send *cube* :newcoords prev-cube-coords)
        (send *cube* :worldcoords)))
    (send *irtviewer* :draw-objects)
    (when av (list (list av) (list tm)))))


(defun hold-lift-down-motion (&key (lift-height nil) (move-robot t) (tm 5000) (use-torso 0.001))
  (let* ((av)
         (prev-av (send *robot* :angle-vector))
         (prev-cube-coords (send *cube* :copy-worldcoords))
         (cube-z (elt (send prev-cube-coords :pos) 2))
         (cube-height (get-cube-height)))
    ;; lift down
    (unless lift-height
      (setq lift-height (- (- cube-z (/ cube-height 2.0)) *table-z*)))
    (send *cube* :translate (float-vector 0 0 (* -1.0 lift-height)) :world)
    (setq av (send *robot* :inverse-kinematics
                   (list
                     (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                           :copy-worldcoords)
                     (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                           :copy-worldcoords))
                   :rotation-axis (list t t)
                   :move-target (list (send *robot* :larm :end-coords)
                                   (send *robot* :rarm :end-coords))
                   :use-torso use-torso))
    (unless av
      (setq av (send *robot* :inverse-kinematics
                     (list
                       (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                             :copy-worldcoords)
                       (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                             :copy-worldcoords))
                     :rotation-axis (list :z :z)
                     :move-target (list (send *robot* :larm :end-coords)
                                        (send *robot* :rarm :end-coords))
                     :use-torso use-torso)))
    (if av
      (when move-robot
        (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
        (send *ri* :wait-interpolation))
      (progn
        (send *robot* :angle-vector prev-av)
        (send *cube* :newcoords prev-cube-coords)
        (send *cube* :worldcoords)))
    (send *irtviewer* :draw-objects)
    (when av (list (list av) (list tm)))))


;; rotate motion

(defun get-rotate-height
  (&key (current-angle) (prev-z) (cube-norm-length)
        (initial-angle 0) (rotate-axis :x) (rotate-type :push))
  (unless (or (eq rotate-axis :x) (eq rotate-axis :y))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from get-rotate-height nil))
  (cond
    ((or (eq rotate-axis :x) (eq rotate-axis :y))
     (* (sin (abs (+ initial-angle current-angle)))
        (/ cube-norm-length 2.0)))
    (t nil)))


(defun rotate-motion
  (&key (rotate-angle pi/2)
        (rotate-axis :x)
        (rotate-type :push)
        (initial-rotate-angle 0)
        (move-robot t)
        (tm 1000)
        (use-torso 0.001)
        (cube-height (get-cube-height))
        (cube-depth (get-cube-depth))
        (cube-width (get-cube-width)))
  (unless (or (eq rotate-axis :x) (eq rotate-axis :y) (eq rotate-axis :z))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from rotate-motion nil))
  (unless (and (send *cube* :get :larm-contact-coords)
               (send *cube* :get :rarm-contact-coords))
    (ros::ros-error "robot is not contacted to cube")
    (return-from rotate-motion nil))
  (let* ((av)
         (rotate-height)
         (prev-rotate-height)
         (rotate-step (if (> rotate-angle 0) 0.1 -0.1))
         (rotate-count 0)
         (max-rotate-count (floor (/ rotate-angle rotate-step)))
         (cube-norm-length
           (cond ((eq rotate-axis :x) (norm (float-vector cube-width cube-height)))
                 ((eq rotate-axis :y) (norm (float-vector cube-depth cube-height)))
                 (t nil)))
         (initial-angle
           (+ initial-rotate-angle
              (* (cond ((equal initial-rotate-angle 0)
                        (if (> rotate-angle 0) 1.0 -1.0))
                       ((> initial-rotate-angle 0) 1.0)
                       (t -1.0))
                 (cond ((eq rotate-axis :x) (atan (/ cube-height cube-width)))
                       ((eq rotate-axis :y) (atan (/ cube-height cube-depth)))
                       (t 0)))))
         (current-angle 0)
         (current-z (elt (send *cube* :worldpos) 2))
         (prev-av (send *robot* :angle-vector))
         (prev-cube-coords (send *cube* :copy-worldcoords))
         (rotate-avs nil)
         (rotate-tms nil))
    (setq prev-rotate-height
          (get-rotate-height :current-angle current-angle
                             :prev-z current-z
                             :cube-norm-length cube-norm-length
                             :initial-angle initial-angle
                             :rotate-axis rotate-axis
                             :rotate-type rotate-type))
    (block :rotate-motion-step
      (while (> max-rotate-count rotate-count)
        (setq current-angle (+ current-angle rotate-step))
        (send *cube* :rotate rotate-step rotate-axis :world)
        (when (or (eq rotate-axis :x) (eq rotate-axis :y))
          (setq rotate-height
                (get-rotate-height :current-angle current-angle
                                   :prev-z current-z
                                   :cube-norm-length cube-norm-length
                                   :initial-angle initial-angle
                                   :rotate-axis rotate-axis
                                   :rotate-type rotate-type))
          (send *cube* :translate
                (float-vector 0 0 (- rotate-height prev-rotate-height)) :world)
          (setq current-z (+ current-z (- rotate-height prev-rotate-height)))
          (setq prev-rotate-height rotate-height))
        (setq av (send *robot* :inverse-kinematics
                       (list
                         (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                               :copy-worldcoords)
                         (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                               :copy-worldcoords))
                       :rotation-axis (list t t)
                       :move-target (list (send *robot* :larm :end-coords)
                                          (send *robot* :rarm :end-coords))
                       :use-torso use-torso))
        (unless av
          (setq av (send *robot* :inverse-kinematics
                         (list
                           (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                                 :copy-worldcoords)
                           (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                                 :copy-worldcoords))
                         :rotation-axis (list :z :z)
                         :move-target (list (send *robot* :larm :end-coords)
                                            (send *robot* :rarm :end-coords))
                         :use-torso use-torso)))
        (send *irtviewer* :draw-objects)
        (if av
          (progn
            (setq rotate-avs (append rotate-avs (list av)))
            (setq rotate-tms (append rotate-tms (list tm))))
          (return-from :rotate-motion-step nil))
        (setq rotate-count (+ rotate-count 1)))
      (send *cube* :rotate (- rotate-angle current-angle) rotate-axis :world)
      (when (or (eq rotate-axis :x) (eq rotate-axis :y))
        (setq current-angle rotate-angle)
        (setq rotate-height
              (get-rotate-height :current-angle current-angle
                                 :prev-z current-z
                                 :cube-norm-length cube-norm-length
                                 :initial-angle initial-angle
                                 :rotate-axis rotate-axis
                                 :rotate-type rotate-type))
        (send *cube* :translate
              (float-vector 0 0 (- rotate-height prev-rotate-height)) :world)
        (setq current-z (+ current-z (- rotate-height prev-rotate-height))))
      (setq av (send *robot* :inverse-kinematics
                     (list
                       (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                             :copy-worldcoords)
                       (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                             :copy-worldcoords))
                     :rotation-axis (list t t)
                     :move-target (list (send *robot* :larm :end-coords)
                                        (send *robot* :rarm :end-coords))
                     :use-torso use-torso))
      (unless av
        (setq av (send *robot* :inverse-kinematics
                       (list
                         (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                               :copy-worldcoords)
                         (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                               :copy-worldcoords))
                       :rotation-axis (list :z :z)
                       :move-target (list (send *robot* :larm :end-coords)
                                          (send *robot* :rarm :end-coords))
                       :use-torso use-torso)))
      (send *irtviewer* :draw-objects)
      (when av
        (setq rotate-avs (append rotate-avs (list av)))
        (setq rotate-tms (append rotate-tms (list tm)))))
    (if av
      (when move-robot
        (send *ri* :angle-vector-sequence rotate-avs rotate-tms nil 0)
        (send *ri* :wait-interpolation))
      (progn
        (send *robot* :angle-vector prev-av)
        (send *cube* :newcoords prev-cube-coords)
        (send *cube* :worldcoords)))
    (send *irtviewer* :draw-objects)
    (when av (list rotate-avs rotate-tms))))


(defun push-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :push) args)))


(defun support-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :support) args)))


;; switch rotate motion

(defun get-switch-rotate-angle (&key (rotate-angle pi/2) (rotate-axis :x) (offset 0.1))
  (let ((cube-height (get-cube-height))
        (cube-width (get-cube-width))
        (cube-depth (get-cube-depth)))
    (* (if (> rotate-angle 0) 1.0 -1.0)
       (- (cond ((eq rotate-axis :x)
                 (- pi/2 (atan (/ cube-height cube-width))))
                ((eq rotate-axis :y)
                 (- pi/2 (atan (/ cube-height cube-depth))))
                ((eq rotate-axis :z) (/ pi 4)))
          offset))))


;; reset-motion

(defun reset-motion (&key (move-robot t) (tm 5000))
  (let ((av (send *robot* :reset-cube-manip-pose)))
    (send *irtviewer* :draw-objects)
    (when move-robot
      ; (send *ri* :stop-grasp)
      (send *ri* :start-grasp :arms :objects nil)
      (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (list (list av) (list tm))))


(defun reset-cube-manip-motion (arm &key (move-robot t) (tm 5000))
  (let ((av (send *robot* :angle-vector))
        (reset-av (send *robot* :reset-cube-manip-pose)))
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
    (send *robot* :angle-vector av)
    (when move-robot
      (send *ri* :angle-vector (send *robot* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (list (list av) (list tm))))


(provide :common-motions "common-motions.l")
