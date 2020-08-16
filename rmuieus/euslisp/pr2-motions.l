(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; parameters
(defvar *cube* (make-cube 350 350 265))
(defvar *cube-pos-x* 450)
(defvar *cube-pos-y* 0)
(defvar *cube-pos-z* 800)
(defvar *cube-prev-lift-coords* nil)

;; init

(defun motion-init ()
  (pr2-init)
  (send *cube* :reset-coords)
  (send *cube* :worldcoords)
  (send *cube* :newcoords
        (make-coords :pos (float-vector *cube-pos-x* *cube-pos-y* *cube-pos-z*)))
  (send *cube* :worldcoords)
  (add-cube-coords)
  (objects (list *pr2* *cube*))
  (reset-motion))


(defun assoc-cube (arm)
  (when (eq arm :arms) (setq arm :larm))
  (send (send *pr2* arm :end-coords :parent) :assoc *cube*))


(defun dissoc-cube (arm)
  (when (eq arm :arms) (setq arm :larm))
  (send (send *pr2* arm :end-coords :parent) :dissoc *cube*))


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
         (upper-cube-coords
           (make-cascoords :name :upper-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 0 (/ cube-height 2.0)) :world)))
         (lower-cube-coords
           (make-cascoords :name :lower-cube-coords
                           :coords (send (send cube-coords :copy-worldcoords) :translate
                                         (float-vector 0 0 (/ cube-height -2.0)) :world))))
    (send *cube* :assoc front-cube-coords)
    (send *cube* :assoc back-cube-coords)
    (send *cube* :assoc left-cube-coords)
    (send *cube* :assoc right-cube-coords)
    (send *cube* :assoc upper-cube-coords)
    (send *cube* :assoc lower-cube-coords)
    (send *cube* :put :front-cube-coords front-cube-coords)
    (send *cube* :put :back-cube-coords back-cube-coords)
    (send *cube* :put :left-cube-coords left-cube-coords)
    (send *cube* :put :right-cube-coords right-cube-coords)
    (send *cube* :put :upper-cube-coords upper-cube-coords)
    (send *cube* :put :lower-cube-coords lower-cube-coords)))


(defun get-cube-length (axis)
  (let* ((cube-coords
           (list (send (send *cube* :get :front-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :back-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :left-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :right-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :upper-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :lower-cube-coords) :copy-worldcoords)))
         (cube-pos-list
           (mapcar #'(lambda (c)
                       (elt (send c :worldpos)
                            (cond ((eq axis :x) 0) ((eq axis :y) 1) ((eq axis :z) 2) (t nil))))
                   cube-coords)))
    (- (reduce #'max cube-pos-list) (reduce #'min cube-pos-list))))


(defun get-cube-depth () (get-cube-length :x))
(defun get-cube-width () (get-cube-length :y))
(defun get-cube-height () (get-cube-length :z))


(defun get-cube-coords (direction)
  (let* ((cube-coords
           (list (send (send *cube* :get :front-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :back-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :left-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :right-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :upper-cube-coords) :copy-worldcoords)
                 (send (send *cube* :get :lower-cube-coords) :copy-worldcoords)))
         (cube-pos-list
           (mapcar #'(lambda (c)
                       (elt (send c :worldpos)
                            (cond ((or (eq direction :front) (eq direction :back)) 0)
                                  ((or (eq direction :left) (eq direction :right)) 1)
                                  ((or (eq direction :upper) (eq direction :lower)) 2)
                                  (t nil))))
                   cube-coords))
         (eval-func (cond ((or (eq direction :front) (eq direction :left) (eq direction :upper)) #'max)
                          ((or (eq direction :back) (eq direction :right) (eq direction :lower)) #'min)
                          (t nil)))
         (eval-value (reduce eval-func cube-pos-list))
         (coords-index (position-if #'(lambda (x) (eq x eval-value)) cube-pos-list)))
    (elt cube-coords coords-index)))


(defun get-front-cube-coords () (get-cube-coords :front))
(defun get-back-cube-coords () (get-cube-coords :back))
(defun get-left-cube-coords () (get-cube-coords :left))
(defun get-right-cube-coords () (get-cube-coords :right))
(defun get-upper-cube-coords () (get-cube-coords :upper))
(defun get-lower-cube-coords () (get-cube-coords :lower))


;; motions
;; push & push-hold

(defun get-push-target-coords
  (arm &key (motion-type :push)
            (push-axis :y)
            (push-position :center)
            (push-direction :left)
            (approach-direction :back)
            (hold-type :tight))
  (when (not (or (eq arm :larm) (eq arm :rarm)))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from get-push-target-coords nil))
  (when (not (or (eq push-axis :x) (eq push-axis :y) (eq push-axis :z)))
    (ros::ros-error (format nil "push-axis is invalid: ~A" push-axis))
    (return-from get-push-target-coords nil))
  (when (not (or (eq hold-type :tight) (eq hold-type :normal) (eq hold-type :light)))
    (ros::ros-error (format nil "hold-type is invalid: ~A" hold-type))
    (return-from get-push-target-coords nil))
  (when (or (and (eq push-axis :x)
                 (not (and (or (eq push-direction :front)
                               (eq push-direction :back))
                           (or (and (or (and (eq arm :larm)
                                             (eq approach-direction :left))
                                        (and (eq arm :rarm)
                                             (eq approach-direction :right)))
                                    (or (eq push-position :upper)
                                        (eq push-position :center)
                                        (eq push-position :lower)))
                               (and (eq approach-direction :upper)
                                    (or (eq push-position :left)
                                        (eq push-position :center)
                                        (eq push-position :right)))))))
            (and (eq push-axis :y)
                 (not (and (or (eq push-direction :left)
                               (eq push-direction :right))
                           (or (and (eq approach-direction :back)
                                    (or (eq push-position :upper)
                                        (eq push-position :center)
                                        (eq push-position :lower)))
                               (and (eq approach-direction :upper)
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
  (let ((hold-offset (cond ((eq hold-type :tight) 50)
                           ((eq hold-type :normal) 0)
                           ((eq hold-type :light) -50)
                           (t nil))))
    (cond
      ((eq push-axis :x)
       (let ((target-coords
               (make-coords :pos (send (send *cube* :copy-worldcoords) :worldpos)
                            :rpy (float-vector 0 0 0)))
             (x-offset (if (eq motion-type :approach) 50 0)))
         (cond
           ((or (and (eq arm :larm) (eq approach-direction :left))
                (and (eq arm :rarm) (eq approach-direction :right)))
            (let ((z-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :upper)
                                   (- (/ (get-cube-height) 2.0) 50))
                                  ((eq push-position :lower)
                                   (- 50 (/ (get-cube-height) 2.0))))))
              (send target-coords :translate
                    (float-vector
                      (* (if (eq push-direction :front) 1.0 -1.0)
                         (+ (/ (get-cube-depth) 2.0) x-offset))
                      (* (if (eq arm :larm) -1.0 1.0) hold-offset)
                      z-offset))
              (send target-coords :rotate (if (eq push-direction :front) -pi/2 pi/2) :y)
              (send target-coords :rotate (if (eq arm :larm) -pi/2 pi/2) :z)))
           ((eq approach-direction :upper)
            (let ((y-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :left)
                                   (- (/ (get-cube-width) 2.0) 50))
                                  ((eq push-position :right)
                                   (- 50 (/ (get-cube-width) 2.0))))))
              (send target-coords :translate
                    (float-vector
                      (* (if (eq push-direction :front) 1.0 -1.0)
                         (+ (/ (get-cube-depth) 2.0) x-offset))
                      y-offset
                      (* -1.0 hold-offset)))
              (send target-coords :rotate pi/2 :y)
              (when (eq push-direction :front)
                (send target-coords :rotate pi :x))))
           (t (return-from get-push-target-coords nil)))
         target-coords))
      ((eq push-axis :y)
       (let ((target-coords
               (make-coords :pos (send (send *cube* :copy-worldcoords) :worldpos)
                            :rpy (float-vector 0 0 0)))
             (y-offset (if (eq motion-type :approach) 50 0)))
         (cond
           ((eq approach-direction :back)
            (let ((z-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :upper)
                                   (- (/ (get-cube-height) 2.0) 50))
                                  ((eq push-position :lower)
                                   (- 50 (/ (get-cube-height) 2.0))))))
              (send target-coords :translate
                    (float-vector
                      hold-offset
                      (* (if (eq push-direction :left) 1.0 -1.0)
                         (+ (/ (get-cube-width) 2.0) y-offset))
                      z-offset))
              (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)))
           ((eq approach-direction :upper)
            (let ((x-offset (cond ((eq push-position :center) 0.0)
                                  ((eq push-position :front)
                                   (- (/ (get-cube-depth) 2.0) 50))
                                  ((eq push-position :back)
                                   (- 50 (/ (get-cube-depth) 2.0))))))
              (send target-coords :translate
                    (float-vector
                      x-offset
                      (* (if (eq push-direction :left) 1.0 -1.0)
                         (+ (/ (get-cube-width) 2.0) y-offset))
                      (* -1.0 hold-offset)))
              (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)
              (send target-coords :rotate (if (eq push-direction :left) -pi/2 pi/2) :z)))
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
            (tm 5000)
            (use-torso 0.001))
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
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
                :hold-type hold-type))
            (rarm-target-coords
              (get-push-target-coords
                :rarm :motion-type motion-type
                :push-axis (cadr push-axes)
                :push-position (cadr push-positions)
                :push-direction (cadr push-directions)
                :approach-direction (cadr approach-directions)
                :hold-type hold-type)))
        (when (or (null larm-target-coords)
                  (null rarm-target-coords))
          (return-from push-motion-step nil))
        (setq av (send *pr2* :inverse-kinematics
                       (list larm-target-coords rarm-target-coords)
                       :move-target (list (send *pr2* :larm :end-coords)
                                          (send *pr2* :rarm :end-coords))
                       :rotation-axis (list t t)
                       :use-torso use-torso))
        (unless av
          (setq av (send *pr2* :inverse-kinematics
                         (list larm-target-coords rarm-target-coords)
                         :move-target (list (send *pr2* :larm :end-coords)
                                            (send *pr2* :rarm :end-coords))
                         :rotation-axis (list :z :z)
                         :use-torso use-torso))))
      (let ((target-coords
              (get-push-target-coords
                arm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions)
                :hold-type hold-type)))
        (when (null target-coords)
          (return-from push-motion-step nil))
        (setq av (send *pr2* arm :inverse-kinematics target-coords
                       :rotation-axis t
                       :use-torso use-torso))
        (unless av
          (setq av (send *pr2* arm :inverse-kinematics target-coords
                         :rotation-axis :z
                         :use-torso use-torso)))))
    (when (and av move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    (list av tm)))


(defun push-motion
  (arm &key (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (hold-type :tight)
            (move-robot t)
            (push-tm 5000)
            (approach-tm 5000)
            (use-torso 0.001))
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from push-motion nil))
  (let ((approached) (pushed))
    (setq approached
          (push-motion-step
            arm :motion-type :approach
            :push-axes push-axes
            :push-positions push-positions
            :push-directions push-directions
            :approach-directions approach-directions
            :hold-type hold-type
            :move-robot move-robot
            :tm approach-tm
            :use-torso use-torso))
    (when approached
      (setq pushed
            (push-motion-step
              arm :motion-type :push
              :push-axes push-axes
              :push-positions push-positions
              :push-directions push-directions
              :approach-directions approach-directions
              :hold-type hold-type
              :move-robot move-robot
              :tm push-tm
              :use-torso use-torso)))
    (when pushed
      (list (list (car approached) (car pushed))
            (list (cadr approached) (cadr pushed))))))


(defun push-hold-motion
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm)
        (hold-type :tight)
        (move-robot t)
        (use-torso 0.001))
  (when (not (or (eq front-arm :larm)
                 (eq front-arm :rarm)))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (return-from push-hold-motion nil))
  (when (not (or (eq upper-arm :larm)
                 (eq upper-arm :rarm)))
    (ros::ros-error (format nil "upper-arm is invalid: ~A" upper-arm))
    (return-from push-hold-motion nil))
  (when (not (or (eq approach-direction :back)
                 (eq approach-direction :upper)
                 (eq approach-direction :side)))
    (ros::ros-error (format nil "approach-direction is invalid: ~A" approach-direction))
    (return-from push-hold-motion nil))
  (let ((push-directions
          (cond ((eq push-axis :x)
                 (if (eq front-arm :larm)
                   (list :front :back) (list :back :front)))
                ((eq push-axis :y) (list :left :right))
                ((eq push-axis :z)
                 (if (eq upper-arm :larm)
                   (list :upper :lower) (list :upper :lower)))))
        (approach-directions
          (cond ((eq approach-direction :side)
                 (list :left :right))
                (t (list approach-direction approach-direction))))
        push-holded)
    (setq push-holded
          (push-motion
            :arms
            :push-axes (list push-axis push-axis)
            :push-positions (list push-position push-position)
            :push-directions push-directions
            :approach-directions approach-directions
            :hold-type hold-type
            :move-robot move-robot
            :use-torso use-torso))
    (when push-holded
      (let ((larm-contact-coords
              (make-cascoords :name :larm-contact-coords
                              :coords (send (send *pr2* :larm :end-coords) :copy-worldcoords)))
            (rarm-contact-coords
              (make-cascoords :name :rarm-contact-coords
                              :coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))))
        (send *cube* :assoc larm-contact-coords)
        (send *cube* :assoc rarm-contact-coords)
        (send *cube* :put :larm-contact-coords larm-contact-coords)
        (send *cube* :put :rarm-contact-coords rarm-contact-coords)))
    push-holded))


;; push-release & push-hold-release

(defun get-push-release-target-coords (arm)
  (let ((target-coords (send (send *pr2* arm :end-coords) :copy-worldcoords)))
    (send target-coords :translate (float-vector 0 0 -50))
    target-coords))


(defun push-release-motion (arm &key (move-robot t) (tm 5000) (use-torso 0.001))
  ;; release
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (ros::ros-error (format nil "arm is invalid: ~A" arm))
    (return-from push-release-motion nil))
  (let ((av))
    (if (eq arm :arms)
      (let ((larm-target-coords (get-push-release-target-coords :larm))
            (rarm-target-coords (get-push-release-target-coords :rarm)))
        (setq av (send *pr2* :inverse-kinematics
                       (list larm-target-coords rarm-target-coords)
                       :move-target (list (send *pr2* :larm :end-coords)
                                          (send *pr2* :rarm :end-coords))
                       :rotation-axis (list t t)
                       :use-torso use-torso))
        (unless av
          (setq av (send *pr2* :inverse-kinematics
                         (list larm-target-coords rarm-target-coords)
                         :move-target (list (send *pr2* :larm :end-coords)
                                            (send *pr2* :rarm :end-coords))
                         :rotation-axis (list :z :z)
                         :use-torso use-torso))))
      (let ((target-coords (get-push-release-target-coords arm)))
        (setq av (send *pr2* arm :inverse-kinematics target-coords
                       :rotation-axis t :use-torso use-torso))
        (unless av
          (setq av (send *pr2* arm :inverse-kinematics target-coords
                         :rotation-axis :z :use-torso use-torso)))))
    (when (and av move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    (when av (list (list av) (list tm)))))


(defun push-hold-release-motion (&key (move-robot t) (tm 5000) (use-torso 0.001))
  (let ((released
          (push-release-motion :arms
                               :move-robot move-robot
                               :tm tm
                               :use-torso use-torso))
        (larm-contact-coords (send *cube* :get :larm-contact-coords))
        (rarm-contact-coords (send *cube* :get :rarm-contact-coords)))
    (when released
      (when larm-contact-coords
        (send *cube* :dissoc larm-contact-coords)
        (send *cube* :put :larm-contact-coords nil))
      (when rarm-contact-coords
        (send *cube* :dissoc rarm-contact-coords)
        (send *cube* :put :rarm-contact-coords nil)))
    (send *irtviewer* :draw-objects)
    released))


(defun hold-lift-up-motion (&key (lift-height 100) (move-robot t) (tm 5000) (use-torso 0.001))
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        (av))
    (setq *cube-prev-lift-coords* (send *cube* :copy-worldcoords))
    ;; lift up
    (send larm-target-coords :translate (float-vector 0 0 lift-height) :world)
    (send rarm-target-coords :translate (float-vector 0 0 lift-height) :world)
    (setq av (send *pr2* :inverse-kinematics
                   (list larm-target-coords rarm-target-coords)
                   :rotation-axis (list t t)
                   :move-target (list (send *pr2* :larm :end-coords)
                                   (send *pr2* :rarm :end-coords))
                   :use-torso use-torso))
    (unless av
      (setq av (send *pr2* :inverse-kinematics
                     (list larm-target-coords rarm-target-coords)
                     :rotation-axis (list :z :z)
                     :move-target (list (send *pr2* :larm :end-coords)
                                        (send *pr2* :rarm :end-coords))
                     :use-torso use-torso)))
    (when av
      (send *cube* :translate (float-vector 0 0 lift-height) :world)
      (send *irtviewer* :draw-objects)
      (when move-robot
        (send *ri* :angle-vector (send *pr2* :angle-vector) tm nil 0)
        (send *ri* :wait-interpolation))
      (list (list av) (list tm)))))


(defun hold-lift-down-motion (&key (lift-height 100) (move-robot t) (tm 5000) (use-torso 0.001))
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        (av))
    ;; lift down
    (send larm-target-coords :translate (float-vector 0 0 (* -1.0 lift-height)) :world)
    (send rarm-target-coords :translate (float-vector 0 0 (* -1.0 lift-height)) :world)
    (setq av (send *pr2* :inverse-kinematics
                   (list larm-target-coords rarm-target-coords)
                   :rotation-axis (list t t)
                   :move-target (list (send *pr2* :larm :end-coords)
                                      (send *pr2* :rarm :end-coords))
                   :use-torso use-torso))
    (when av
      (send *cube* :newcoords *cube-prev-lift-coords*)
      (send *irtviewer* :draw-objects)
      (when move-robot
        (send *ri* :angle-vector (send *pr2* :angle-vector) tm nil 0)
        (send *ri* :wait-interpolation))
      (list (list av) (list tm)))))


;; rotate motion

(defun get-rotate-height
  (&key (current-angle) (prev-height) (cube-norm-length)
        (initial-angle 0)(rotate-axis :x) (rotate-type :push))
  (when (not (or (eq rotate-axis :x) (eq rotate-axis :y)))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from get-rotate-height nil))
  (cond
    ((eq rotate-type :push)
     (cond
       ((eq rotate-axis :x)
        (- (* (sin (+ initial-angle (abs current-angle))) (/ cube-norm-length 2.0))
           prev-height))
       ((eq rotate-axis :y)
        (- (* (sin (+ initial-angle (abs current-angle))) (/ cube-norm-length 2.0))
           prev-height))))
    ((eq rotate-type :support)
     ;; TODO: add support rotate
     )
    (t nil)))


(defun rotate-motion
  (&key (rotate-angle pi/2) (rotate-axis :x) (front-arm :larm) (rotate-type :push)
        (move-robot t) (tm 1000) (use-torso 0.001))
  (when (not (or (eq rotate-axis :x) (eq rotate-axis :y) (eq rotate-axis :z)))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from rotate-motion nil))
  (let* ((av)
         (rotate-height)
         (rotate-step (if (> rotate-angle 0) 0.1 -0.1))
         (rotate-count 0)
         (max-rotate-count (floor (/ rotate-angle rotate-step)))
         (cube-height (get-cube-height))
         (cube-depth (get-cube-depth))
         (cube-width (get-cube-width))
         (cube-norm-length
           (cond ((eq rotate-axis :x) (norm (float-vector cube-width cube-height)))
                 ((eq rotate-axis :y) (norm (float-vector cube-depth cube-height)))
                 (t nil)))
         (initial-angle
           (cond ((eq rotate-axis :x) (atan (/ cube-height cube-width)))
                 ((eq rotate-axis :y) (atan (/ cube-height cube-depth)))
                 (t nil)))
         (current-angle 0)
         (current-height (/ cube-height 2.0))
         (prev-av (send *pr2* :angle-vector))
         (prev-cube-coords (send *cube* :copy-worldcoords))
         (rotate-avs nil)
         (rotate-tms nil))
    (block :rotate-motion-step
      (while (> max-rotate-count rotate-count)
        (setq current-angle (+ current-angle rotate-step))
        (send *cube* :rotate rotate-step rotate-axis :world)
        (when (or (eq rotate-axis :x) (eq rotate-axis :y))
          (setq rotate-height
                (get-rotate-height :current-angle current-angle
                                   :prev-height current-height
                                   :cube-norm-length cube-norm-length
                                   :initial-angle initial-angle
                                   :rotate-axis rotate-axis
                                   :rotate-type rotate-type))
          (send *cube* :translate (float-vector 0 0 rotate-height) :world)
          (setq current-height (+ current-height rotate-height)))
        (setq av (send *pr2* :inverse-kinematics
                       (list
                         (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                               :copy-worldcoords)
                         (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                               :copy-worldcoords))
                       :rotation-axis (list t t)
                       :move-target (list (send *pr2* :larm :end-coords)
                                          (send *pr2* :rarm :end-coords))
                       :use-torso use-torso))
        (unless av
          (setq av (send *pr2* :inverse-kinematics
                         (list
                           (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                                 :copy-worldcoords)
                           (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                                 :copy-worldcoords))
                         :rotation-axis (list :z :z)
                         :move-target (list (send *pr2* :larm :end-coords)
                                            (send *pr2* :rarm :end-coords))
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
                                 :prev-height current-height
                                 :cube-norm-length cube-norm-length
                                 :initial-angle initial-angle
                                 :rotate-axis rotate-axis
                                 :rotate-type rotate-type))
        (send *cube* :translate (float-vector 0 0 rotate-height) :world)
        (setq current-height (+ current-height rotate-height)))
      (setq av (send *pr2* :inverse-kinematics
                     (list
                       (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                             :copy-worldcoords)
                       (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                             :copy-worldcoords))
                     :rotation-axis (list t t)
                     :move-target (list (send *pr2* :larm :end-coords)
                                        (send *pr2* :rarm :end-coords))
                     :use-torso use-torso))
      (unless av
        (setq av (send *pr2* :inverse-kinematics
                       (list
                         (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                               :copy-worldcoords)
                         (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                               :copy-worldcoords))
                       :rotation-axis (list :z :z)
                       :move-target (list (send *pr2* :larm :end-coords)
                                          (send *pr2* :rarm :end-coords))
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
        (send *pr2* :angle-vector prev-av)
        (send *cube* :newcoords prev-cube-coords)
        (send *cube* :worldcoords)))
    (send *irtviewer* :draw-objects)
    (when av (list rotate-avs rotate-tms))))


(defun push-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :push) args)))


(defun support-rotate-motion (&rest args)
  (apply #'rotate-motion (append (list :rotate-type :support) args)))


;; reset-motion

(defun reset-motion (&key (move-robot t) (tm 5000))
  (let ((av (send *pr2* :reset-manip-pose)))
    (send *irtviewer* :draw-objects)
    (when move-robot
      (send *ri* :angle-vector (send *pr2* :angle-vector) tm nil 0)
      (send *ri* :wait-interpolation))
    (list (list av) (list tm))))


(provide :pr2-motions "pr2-motions.l")
