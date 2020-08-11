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
                       (elt (send c :worldpos) (cond ((eq axis :x) 0) ((eq axis :y) 1) (t 2))))
                   cube-coords)))
    (- (reduce #'max cube-pos-list) (reduce #'min cube-pos-list))))


(defun get-cube-depth () (get-cube-length :x))
(defun get-cube-width () (get-cube-length :y))
(defun get-cube-height () (get-cube-length :z))


;; motions
;; push & push-hold

(defun get-push-target-coords
  (arm &key (motion-type :push)
            (push-axis :y)
            (push-position :center)
            (push-direction :left)
            (approach-direction :back))

  (when (not (or (eq arm :larm) (eq arm :rarm)))
    (return-from get-push-target-coords nil))
  (when (not (or (eq push-axis :x) (eq push-axis :y) (eq push-axis :z)))
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
    (return-from get-push-target-coords nil))
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
                    (if (eq arm :larm) -100 100)
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
                    -100))
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
                    100
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
                    -100))
            (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)
            (send target-coords :rotate (if (eq push-direction :left) -pi/2 pi/2) :z)))
         (t (return-from get-push-target-coords nil)))
       target-coords))
    (t (return-from get-push-target-coords nil))))


(defun push-motion-step
  (arm &key (motion-type :push)
            (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (move-robot t))

  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-motion-step nil))
  ;; push
  (let ((ik-solved))
    (if (eq arm :arms)
      (let ((larm-target-coords
              (get-push-target-coords
                :larm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions)))
            (rarm-target-coords
              (get-push-target-coords
                :rarm :motion-type motion-type
                :push-axis (cadr push-axes)
                :push-position (cadr push-positions)
                :push-direction (cadr push-directions)
                :approach-direction (cadr approach-directions))))
        (when (or (null larm-target-coords)
                  (null rarm-target-coords))
          (return-from push-motion-step nil))
        (setq ik-solved
              (send *pr2* :inverse-kinematics
                    (list larm-target-coords rarm-target-coords)
                    :move-target (list (send *pr2* :larm :end-coords)
                                       (send *pr2* :rarm :end-coords))
                    :rotation-axis (list t t)
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (list larm-target-coords rarm-target-coords)
                      :move-target (list (send *pr2* :larm :end-coords)
                                         (send *pr2* :rarm :end-coords))
                      :rotation-axis (list :z :z)
                      :use-torso 0.001))))
      (let ((target-coords
              (get-push-target-coords
                arm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions))))
        (when (null target-coords)
          (return-from push-motion-step nil))
        (setq ik-solved
              (send *pr2* arm :inverse-kinematics target-coords
                    :rotation-axis t
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* arm :inverse-kinematics target-coords
                      :rotation-axis :z
                      :use-torso 0.001)))))
    (when (and ik-solved move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
            (cond ((eq arm :larm) :larm-controller)
                  ((eq arm :rarm) :rarm-controller)
                  (t nil))
            0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    ik-solved))


(defun push-motion
  (arm &key (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back))
            (move-robot t))

  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-motion nil))
  (let ((approached) (pushed))
    (setq approached
          (push-motion-step
            arm :motion-type :approach
            :push-axes push-axes
            :push-positions push-positions
            :push-directions push-directions
            :approach-directions approach-directions
            :move-robot move-robot))
    (when approached
      (setq pushed
            (push-motion-step
              arm :motion-type :push
              :push-axes push-axes
              :push-positions push-positions
              :push-directions push-directions
              :approach-directions approach-directions
              :move-robot move-robot))
      (list approached pushed))))


(defun push-hold-motion
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm)
        (move-robot t))

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
            :move-robot move-robot))
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


(defun push-release-motion (arm &key (move-robot t))
  ;; release
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-release-motion nil))
  (let ((ik-solved))
    (if (eq arm :arms)
      (let ((larm-target-coords (get-push-release-target-coords :larm))
            (rarm-target-coords (get-push-release-target-coords :rarm)))
        (setq ik-solved
              (send *pr2* :inverse-kinematics
                    (list larm-target-coords rarm-target-coords)
                    :move-target (list (send *pr2* :larm :end-coords)
                                       (send *pr2* :rarm :end-coords))
                    :rotation-axis (list t t)
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (list larm-target-coords rarm-target-coords)
                      :move-target (list (send *pr2* :larm :end-coords)
                                         (send *pr2* :rarm :end-coords))
                      :rotation-axis (list :z :z)
                      :use-torso 0.001))))
      (let ((target-coords (get-push-release-target-coords arm)))
        (setq ik-solved
              (send *pr2* arm :inverse-kinematics
                    target-coords :rotation-axis t :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* arm :inverse-kinematics
                      target-coords :rotation-axis :z :use-torso 0.001)))))
    (when (and ik-solved move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
            (cond ((eq arm :larm) :larm-controller)
                  ((eq arm :rarm) :rarm-controller)
                  (t nil))
            0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    ik-solved))


(defun push-hold-release-motion (&key (move-robot t))
  (let ((released (push-release-motion :arms :move-robot move-robot))
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


(defun hold-lift-up-motion (&key (move-robot t))
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        ik-solved)
    (setq *cube-prev-lift-coords* (send *cube* :copy-worldcoords))
    ;; assoc
    (assoc-cube :arms)
    ;; lift up
    (send larm-target-coords :translate (float-vector 0 0 100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 100) :world)
    (setq ik-solved
          (send *pr2* :inverse-kinematics
                (list larm-target-coords rarm-target-coords)
                :rotation-axis (list t t)
                :move-target (list (send *pr2* :larm :end-coords)
                                   (send *pr2* :rarm :end-coords))
                :use-torso 0.001))
    (unless ik-solved
      (setq ik-solved
            (send *pr2* :inverse-kinematics
                  (list larm-target-coords rarm-target-coords)
                  :rotation-axis (list :z :z)
                  :move-target (list (send *pr2* :larm :end-coords)
                                     (send *pr2* :rarm :end-coords))
                  :use-torso 0.001)))
    ;; disssoc
    (dissoc-cube :arms)
    (when (and ik-solved move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
      (send *ri* :wait-interpolation))
    (send *irtviewer* :draw-objects)
    ik-solved))


(defun hold-lift-down-motion (&key (move-robot t))
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        ik-solved)
    ;; assoc
    (assoc-cube :arms)
    ;; lift down
    (send larm-target-coords :translate (float-vector 0 0 -100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 -100) :world)
    (setq ik-solved
          (send *pr2* :inverse-kinematics
                (list larm-target-coords rarm-target-coords)
                :rotation-axis (list t t)
                :move-target (list (send *pr2* :larm :end-coords)
                                   (send *pr2* :rarm :end-coords))
                :use-torso 0.001))
    ;; disssoc
    (dissoc-cube :arms)
    (when (and ik-solved move-robot)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
      (send *ri* :wait-interpolation))
    (when ik-solved (send *cube* :newcoords *cube-prev-lift-coords*))
    (send *irtviewer* :draw-objects)
    ik-solved))


;; rotate

(defun rotate-motion (&key (rotate-angle pi/2) (rotate-axis :x) (move-robot t))
  (when (not (or (eq rotate-axis :x)
                 (eq rotate-axis :z)))
    (ros::ros-error (format nil "rotate-axis is invalid: ~A" rotate-axis))
    (return-from rotate-motion nil))
  (let* ((ik-solved)
         (move-distance)
         (rotate-step (if (> rotate-angle 0) 0.1 -0.1))
         (rotate-count 0)
         (max-rotate-count (floor (/ rotate-angle rotate-step)))
         (current-angle 0)
         (current-distance (/ (get-cube-height) 2.0))
         (prev-av (send *pr2* :angle-vector))
         (prev-cube-coords (send *cube* :copy-worldcoords))
         (rotate-avs
           (push-hold-motion
             :push-axis :y
             :push-position :center
             :approach-direction (if (eq rotate-axis :x) :back :upper)
             :move-robot nil))
         (rotate-tms (list 5000)))
    (when rotate-avs
      (block :rotate-motion-step
             (while (> max-rotate-count rotate-count)
               (setq current-angle (+ current-angle rotate-step))
               (send *cube* :rotate rotate-step rotate-axis :world)
               (when (eq rotate-axis :x)
                 (setq move-distance
                   (- (* (sin (+ (atan (/ (get-cube-height) (get-cube-width)))
                                 (abs current-angle)))
                         (/ (norm (float-vector (get-cube-width) (get-cube-height))) 2.0))
                      current-distance))
                 (send *cube* :translate (float-vector 0 0 move-distance) :world)
                 (setq current-distance (+ current-distance move-distance)))
               (setq ik-solved
                     (send *pr2* :inverse-kinematics
                           (list
                             (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                                   :copy-worldcoords)
                             (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                                   :copy-worldcoords))
                           :rotation-axis (list t t)
                           :move-target (list (send *pr2* :larm :end-coords)
                                              (send *pr2* :rarm :end-coords))
                           :use-torso 0.001))
               (send *irtviewer* :draw-objects)
               (if ik-solved
                 (progn
                   (setq rotate-avs (append rotate-avs (list ik-solved)))
                   (setq rotate-tms (append rotate-tms (list 1000))))
                 (return-from :rotate-motion-step nil))
               (setq rotate-count (+ rotate-count 1)))
             (send *cube* :rotate (- rotate-angle current-angle) rotate-axis :world)
             (when (eq rotate-axis :x)
               (setq move-distance
                 (- (* (sin (+ (atan (/ (get-cube-height) (get-cube-width)))
                               (abs rotate-angle)))
                       (/ (norm (float-vector (get-cube-width) (get-cube-height))) 2.0))
                    current-distance))
               (send *cube* :translate (float-vector 0 0 move-distance) :world)
               (setq current-distance (+ current-distance move-distance)))
             (setq ik-solved
                   (send *pr2* :inverse-kinematics
                         (list
                           (send (send (send *cube* :get :larm-contact-coords) :worldcoords)
                                 :copy-worldcoords)
                           (send (send (send *cube* :get :rarm-contact-coords) :worldcoords)
                                 :copy-worldcoords))
                         :rotation-axis (list t t)
                         :move-target (list (send *pr2* :larm :end-coords)
                                            (send *pr2* :rarm :end-coords))
                         :use-torso 0.001))
             (if ik-solved
               (progn
                 (setq rotate-avs (append rotate-avs (list ik-solved)))
                 (setq rotate-tms (append rotate-tms (list 1000))))
               (return-from :rotate-motion-step nil))
             (setq ik-solved (push-hold-release-motion :move-robot nil))
             (when ik-solved
               (setq rotate-avs (append rotate-avs (list ik-solved)))
               (setq rotate-tms (append rotate-tms (list 5000)))))
      (if ik-solved
        (when move-robot
          (send *ri* :angle-vector-sequence rotate-avs rotate-tms nil 0)
          (send *ri* :wait-interpolation))
        (progn
          (send *pr2* :angle-vector prev-av)
          (send *cube* :newcoords prev-cube-coords)
          (send *cube* :worldcoords)))
      (send *irtviewer* :draw-objects)
      (when ik-solved rotate-avs))))


;; reset-motion

(defun reset-motion ()
  (send *pr2* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
  (send *ri* :wait-interpolation))


(provide :pr2-motions "pr2-motions.l")
