#!/usr/bin/env roseus

(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")

(ros::load-ros-manifest "std_msgs")


;; parameters

(defvar *device* (make-cube 120 120 120))
(defvar *device-frame-id* "rmui_link")
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)
(defvar *world-frame-id* "world")
(defvar *waiting-visualize-topic-name* "/rmui0/rmui_node/output/status/waiting")
(defvar *moving-visualize-topic-name* "/rmui0/rmui_node/output/status/moving")


;; conditions

(with-open-file
  (larm-get-motion-symbol-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbol/larm-get-motion-symbol-conditions.l"))
  (setq *larm-get-motion-symbol-conditions* (read larm-get-motion-symbol-conditions-f)))
(with-open-file
  (rarm-get-motion-symbol-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbol/rarm-get-motion-symbol-conditions.l"))
  (setq *rarm-get-motion-symbol-conditions* (read rarm-get-motion-symbol-conditions-f)))
(with-open-file
  (dualarm-get-motion-symbol-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbol/dualarm-get-motion-symbol-conditions.l"))
  (setq *dualarm-get-motion-symbol-conditions* (read dualarm-get-motion-symbol-conditions-f)))


;; initial conditions

(setq *object-contact-states* (list :released :released :bottom-contact))
(setq *object-location-state* :center)
(setq *motion-symbol-queue* (list nil nil))


;; functions

(defun get-motion-symbols (contact-positions)
  (let* ((larm-motion-symbols
           (get-motion-symbol :larm contact-positions))
         (rarm-motion-symbols
           (get-motion-symbol :rarm contact-positions)))
    (list larm-motion-symbols rarm-motion-symbols)))


(defun get-motion-symbol (arm contact-positions)
  (let ((contact-surfaces (mapcar #'car contact-positions))
        (bottom-contact-positions
          (remove-if #'null
                     (mapcar #'(lambda (x) (if (equal (car x) :bottom) (cadr x) nil))
                             contact-positions)))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (object-location-state *object-location-state*)
        (arm-conditions (cond ((equal arm :larm) *larm-get-motion-symbol-conditions*)
                              ((equal arm :rarm) *rarm-get-motion-symbol-conditions*)
                              (t nil)))
        (dualarm-conditions *dualarm-get-motion-symbol-conditions*))
    (ros::publish *waiting-visualize-topic-name*
                  (instance std_msgs::Float32 :init :data 0.0))
    (ros::publish *moving-visualize-topic-name*
                  (instance std_msgs::Float32 :init :data 0.0))
    (eval (append '(cond) arm-conditions dualarm-conditions
                  '(((null contact-positions) nil) (t nil))))))


(defun execute-motion-symbols (motion-symbols &key (move-robot t) (use-torso 0.001))
  (let ((larm-motion-symbol-queue (car *motion-symbol-queue*))
        (rarm-motion-symbol-queue (cadr *motion-symbol-queue*))
        (larm-motion-symbol (car motion-symbols))
        (rarm-motion-symbol (cadr motion-symbols))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        larm-execute-p rarm-execute-p)
    ;; check motion symbol queue
    (if (or larm-motion-symbol rarm-motion-symbol)
      (ros::publish *waiting-visualize-topic-name*
                    (instance std_msgs::Float32 :init :data 1.0)))
    (setq larm-motion-symbol-queue
          (append
            (if (>= (length larm-motion-symbol-queue) *motion-symbol-length*)
              (cdr larm-motion-symbol-queue) larm-motion-symbol-queue)
            (list larm-motion-symbol)))
    (setq rarm-motion-symbol-queue
          (append
            (if (>= (length rarm-motion-symbol-queue) *motion-symbol-length*)
              (cdr rarm-motion-symbol-queue) rarm-motion-symbol-queue)
            (list rarm-motion-symbol)))
    (setq larm-execute-p
          (and (>= (length larm-motion-symbol-queue) *motion-symbol-length*)
               (every #'(lambda (x) (equal x larm-motion-symbol)) larm-motion-symbol-queue)))
    (setq rarm-execute-p
          (and (>= (length rarm-motion-symbol-queue) *motion-symbol-length*)
               (every #'(lambda (x) (equal x rarm-motion-symbol)) rarm-motion-symbol-queue)))
    (setq larm-motion-symbol (if larm-execute-p larm-motion-symbol))
    (setq rarm-motion-symbol (if rarm-execute-p rarm-motion-symbol))
    (setq *motion-symbol-queue* (list larm-motion-symbol-queue rarm-motion-symbol-queue))

    ;; execute motion symbols
    (if (or larm-motion-symbol rarm-motion-symbol)
      (progn
        (if move-robot
          (progn
            (ros::publish *waiting-visualize-topic-name*
                          (instance std_msgs::Float32 :init :data 1.0))
            (ros::publish *moving-visualize-topic-name*
                          (instance std_msgs::Float32 :init :data 1.0))))
        (if (equal larm-motion-symbol rarm-motion-symbol)
          (cond
            ((or (and (equal larm-motion-symbol :push-hold-up)
                      (equal rarm-motion-symbol :push-hold-up))
                 (and (equal larm-motion-symbol :support-hold-up)
                      (equal rarm-motion-symbol :support-hold-up)))
             (assoc-cube :arms)
             (hold-lift-up-motion :lift-height 200 :move-robot move-robot :use-torso use-torso)
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ((or (and (equal larm-motion-symbol :push-hold-down)
                      (equal rarm-motion-symbol :push-hold-down))
                 (and (equal larm-motion-symbol :support-hold-down)
                      (equal rarm-motion-symbol :support-hold-down)))
             (assoc-cube :arms)
             (hold-lift-down-motion :move-robot move-robot :use-torso use-torso)
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; push rotate ccw x45
            ((and (equal larm-motion-symbol :push-rotate-ccw-x45)
                  (equal rarm-motion-symbol :push-rotate-ccw-x45))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle
                      (if (and (equal larm-contact-state :bottom-left-contact)
                               (equal rarm-contact-state :top-right-contact))
                        (/ -pi 4.0) 0))
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                                   :initial-rotate-angle initial-rotate-angle
                                   :cube-height cube-height
                                   :cube-width cube-width
                                   :cube-depth cube-depth
                                   :move-robot move-robot
                                   :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; push rotate cw x45
            ((and (equal larm-motion-symbol :push-rotate-cw-x45)
                  (equal rarm-motion-symbol :push-rotate-cw-x45))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle
                      (if (and (equal larm-contact-state :top-left-contact)
                               (equal rarm-contact-state :bottom-right-contact))
                        (/ pi 4.0) 0))
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                                   :initial-rotate-angle initial-rotate-angle
                                   :cube-height cube-height
                                   :cube-width cube-width
                                   :cube-depth cube-depth
                                   :move-robot move-robot
                                   :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; lift push rotate ccw x90
            ((or (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
                      (equal rarm-motion-symbol :lift-push-rotate-ccw-x90))
                 (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-ccw-x90)))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle 0)
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (push-rotate-motion :rotate-angle (/ pi 2.0) :rotate-axis :x
                                   :initial-rotate-angle initial-rotate-angle
                                   :cube-height cube-height
                                   :cube-width cube-width
                                   :cube-depth cube-depth
                                   :move-robot move-robot
                                   :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; lift push rotate cw x90
            ((or (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
                      (equal rarm-motion-symbol :lift-push-rotate-cw-x90))
                 (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-cw-x90)))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle 0)
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (push-rotate-motion :rotate-angle (/ -pi 2.0) :rotate-axis :x
                                   :initial-rotate-angle initial-rotate-angle
                                   :cube-height cube-height
                                   :cube-width cube-width
                                   :cube-depth cube-depth
                                   :move-robot move-robot
                                   :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; push rotate ccw z45
            ((and (equal larm-motion-symbol :push-rotate-ccw-z45)
                  (equal rarm-motion-symbol :push-rotate-ccw-z45))
             (assoc-cube :arms)
             (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :z
                                 :move-robot move-robot :use-torso use-torso)
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ;; push rotate cw z45
            ((and (equal larm-motion-symbol :push-rotate-cw-z45)
                  (equal rarm-motion-symbol :push-rotate-cw-z45))
             (assoc-cube :arms)
             (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :z
                                 :move-robot move-robot :use-torso use-torso)
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ((and (equal larm-motion-symbol :support-rotate-ccw-x45)
                  (equal rarm-motion-symbol :support-rotate-ccw-x45))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle
                      (if (and (equal larm-contact-state :top-left-contact)
                               (equal rarm-contact-state :top-right-contact))
                        (/ pi 4.0) 0))
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (support-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                                      :initial-rotate-angle initial-rotate-angle
                                      :cube-height cube-height
                                      :cube-width cube-width
                                      :cube-depth cube-depth
                                      :move-robot move-robot
                                      :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            ((and (equal larm-motion-symbol :support-rotate-cw-x45)
                  (equal rarm-motion-symbol :support-rotate-cw-x45))
             (assoc-cube :arms)
             (let* ((initial-rotate-angle
                      (if (and (equal larm-contact-state :top-left-contact)
                               (equal rarm-contact-state :top-right-contact))
                        (/ -pi 4.0) 0))
                    (cube-rpy (float-vector 0 0 initial-rotate-angle))
                    (cube-height (get-cube-height :cube-rpy cube-rpy))
                    (cube-width (get-cube-width :cube-rpy cube-rpy))
                    (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
               (support-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                                      :initial-rotate-angle initial-rotate-angle
                                      :cube-height cube-height
                                      :cube-width cube-width
                                      :cube-depth cube-depth
                                      :move-robot move-robot
                                      :use-torso use-torso))
             (dissoc-cube :arms)
             (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
            (t nil)))
        (if larm-motion-symbol
          (cond
            ((equal larm-motion-symbol :left-push)
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
             (return-from execute-motion-symbols (list larm-motion-symbol nil)))
            (t nil)))
        (if rarm-motion-symbol
          (cond
            ((equal rarm-motion-symbol :right-push)
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
             (return-from execute-motion-symbols (list nil rarm-motion-symbol)))
            (t nil)))))))


(defun update-object-contact-state (motion-symbols)
  (unless motion-symbols (return-from update-object-contact-state nil))
  (let ((larm-motion-symbol (car motion-symbols))
        (rarm-motion-symbol (cadr motion-symbols))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (object-location-state *object-location-state*))
    (setq larm-contact-state
        (cond
          ((or (and (equal larm-motion-symbol :left-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :push-rotate-ccw-x45)
                    (equal larm-contact-state :bottom-left-contact))
               (and (equal larm-motion-symbol :push-rotate-cw-x45)
                    (equal larm-contact-state :top-left-contact))
               (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
                    (equal larm-contact-state :bottom-contact))
               (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
                    (equal larm-contact-state :top-contact))
               (and (equal larm-motion-symbol :push-rotate-ccw-z45)
                    (equal larm-contact-state :front-left-contact))
               (and (equal larm-motion-symbol :push-rotate-cw-z45)
                    (equal larm-contact-state :back-left-contact))
               (and (equal larm-motion-symbol :support-rotate-cw-x45)
                    (equal larm-contact-state :top-left-contact)))
           :left-contact)
          ((and (equal larm-motion-symbol :front-push)
                (equal larm-contact-state :released))
           :front-contact)
          ((and (equal larm-motion-symbol :back-push)
                (equal larm-contact-state :released))
           :back-contact)
          ((or (and (equal larm-motion-symbol :top-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :support-rotate-ccw-x45)
                    (equal larm-contact-state :top-left-contact))
               (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
                    (equal larm-contact-state :left-contact)))
           :top-contact)
          ((or (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
                    (equal larm-contact-state :left-contact)))
           :bottom-contact)
          ((or (and (equal larm-motion-symbol :bottom-left-side-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
                    (equal larm-contact-state :left-top-side-contact)
                    (equal object-location-state :left)))
           :bottom-left-side-contact)
          ((or (and (equal larm-motion-symbol :top-right-side-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
                    (equal larm-contact-state :left-top-side-contact)
                    (equal object-location-state :right)))
           :top-right-side-contact)
          ((or (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
                    (equal larm-contact-state :bottom-left-side-contact))
               (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
                    (equal larm-contact-state :top-right-side-contact)))
           :left-top-side-contact)
          ((or (and (equal larm-motion-symbol :top-left-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :push-rotate-ccw-x45)
                    (equal larm-contact-state :left-contact))
               (and (equal larm-motion-symbol :support-rotate-ccw-x45)
                    (equal larm-contact-state :left-contact))
               (and (equal larm-motion-symbol :support-rotate-cw-x45)
                    (equal larm-contact-state :top-contact)))
           :top-left-contact)
          ((or (and (equal larm-motion-symbol :bottom-left-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :push-rotate-cw-x45)
                    (equal larm-contact-state :left-contact)))
           :bottom-left-contact)
          ((or (and (equal larm-motion-symbol :back-left-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :push-rotate-ccw-z45)
                    (equal larm-contact-state :left-contact)))
           :back-left-contact)
          ((or (and (equal larm-motion-symbol :front-left-push)
                    (equal larm-contact-state :released))
               (and (equal larm-motion-symbol :push-rotate-cw-z45)
                    (equal larm-contact-state :left-contact)))
           :front-left-contact)
          ((and (equal larm-motion-symbol :push-release)
                (or (equal larm-contact-state :left-contact)
                    (equal larm-contact-state :front-contact)
                    (equal larm-contact-state :front-left-contact)
                    (equal larm-contact-state :back-contact)
                    (equal larm-contact-state :back-left-contact)
                    (equal larm-contact-state :top-contact)
                    (equal larm-contact-state :top-left-contact)
                    (equal larm-contact-state :bottom-left-contact)
                    (equal larm-contact-state :bottom-left-side-contact)
                    (equal larm-contact-state :top-right-side-contact)
                    (equal larm-contact-state :left-top-side-contact)))
           :released)
          (t larm-contact-state)))
    (setq rarm-contact-state
        (cond
          ((or (and (equal rarm-motion-symbol :right-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :push-rotate-ccw-x45)
                    (equal rarm-contact-state :top-right-contact))
               (and (equal rarm-motion-symbol :push-rotate-cw-x45)
                    (equal rarm-contact-state :bottom-right-contact))
               (and (equal rarm-motion-symbol :lift-push-rotate-ccw-x90)
                    (equal rarm-contact-state :top-contact))
               (and (equal rarm-motion-symbol :lift-push-rotate-cw-x90)
                    (equal rarm-contact-state :bottom-contact))
               (and (equal rarm-motion-symbol :push-rotate-ccw-z45)
                    (equal rarm-contact-state :back-right-contact))
               (and (equal rarm-motion-symbol :push-rotate-cw-z45)
                    (equal rarm-contact-state :front-right-contact))
               (and (equal rarm-motion-symbol :support-rotate-ccw-x45)
                    (equal rarm-contact-state :top-right-contact)))
           :right-contact)
          ((and (equal rarm-motion-symbol :front-push)
                (equal rarm-contact-state :released))
           :front-contact)
          ((and (equal rarm-motion-symbol :back-push)
                (equal rarm-contact-state :released))
           :back-contact)
          ((or (and (equal rarm-motion-symbol :top-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :support-rotate-cw-x45)
                    (equal rarm-contact-state :top-right-contact))
               (and (equal rarm-motion-symbol :lift-push-rotate-cw-x90)
                    (equal rarm-contact-state :right-contact)))
           :top-contact)
          ((or (and (equal rarm-motion-symbol :lift-push-rotate-ccw-x90)
                    (equal rarm-contact-state :right-contact)))
           :bottom-contact)
          ((or (and (equal rarm-motion-symbol :bottom-right-side-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :side-push-rotate-ccw-x90)
                    (equal rarm-contact-state :right-top-side-contact)
                    (equal object-location-state :right)))
           :bottom-right-side-contact)
          ((or (and (equal rarm-motion-symbol :top-left-side-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :side-push-rotate-cw-x90)
                    (equal rarm-contact-state :right-top-side-contact)
                    (equal object-location-state :left)))
           :top-left-side-contact)
          ((or (and (equal rarm-motion-symbol :side-push-rotate-ccw-x90)
                    (equal rarm-contact-state :top-left-side-contact))
               (and (equal rarm-motion-symbol :side-push-rotate-cw-x90)
                    (equal rarm-contact-state :bottom-right-side-contact)))
           :right-top-side-contact)
          ((or (and (equal rarm-motion-symbol :top-right-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :push-rotate-cw-x45)
                    (equal rarm-contact-state :right-contact))
               (and (equal rarm-motion-symbol :support-rotate-ccw-x45)
                    (equal rarm-contact-state :top-contact))
               (and (equal rarm-motion-symbol :support-rotate-cw-x45)
                    (equal rarm-contact-state :right-contact)))
           :top-right-contact)
          ((or (and (equal rarm-motion-symbol :bottom-right-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :push-rotate-ccw-x45)
                    (equal rarm-contact-state :right-contact)))
           :bottom-right-contact)
          ((or (and (equal rarm-motion-symbol :front-right-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :push-rotate-ccw-z45)
                    (equal rarm-contact-state :right-contact)))
           :front-right-contact)
          ((or (and (equal rarm-motion-symbol :back-right-push)
                    (equal rarm-contact-state :released))
               (and (equal rarm-motion-symbol :push-rotate-cw-z45)
                    (equal rarm-contact-state :right-contact)))
           :back-right-contact)
          ((and (equal rarm-motion-symbol :push-release)
                (or (equal rarm-contact-state :right-contact)
                    (equal rarm-contact-state :front-contact)
                    (equal rarm-contact-state :front-right-contact)
                    (equal rarm-contact-state :back-contact)
                    (equal rarm-contact-state :back-right-contact)
                    (equal rarm-contact-state :top-contact)
                    (equal rarm-contact-state :top-right-contact)
                    (equal rarm-contact-state :bottom-right-contact)
                    (equal rarm-contact-state :bottom-right-side-contact)
                    (equal rarm-contact-state :top-left-side-contact)
                    (equal rarm-contact-state :right-top-side-contact)))
           :released)
          (t rarm-contact-state)))
    (setq floor-contact-state
          (cond
            ((or (and (equal floor-contact-state :bottom-contact)
                      (or (and (equal larm-motion-symbol :push-rotate-ccw-x45)
                               (equal rarm-motion-symbol :push-rotate-ccw-x45))
                          (and (equal larm-motion-symbol :push-rotate-cw-x45)
                               (equal rarm-motion-symbol :push-rotate-cw-x45))
                          (and (equal larm-motion-symbol :support-rotate-ccw-x45)
                               (equal rarm-motion-symbol :support-rotate-ccw-x45))
                          (and (equal larm-motion-symbol :support-rotate-cw-x45)
                               (equal rarm-motion-symbol :support-rotate-cw-x45))))
                 (and (or (equal floor-contact-state :bottom-contact)
                          (equal floor-contact-state :bottom-left-side-contact)
                          (equal floor-contact-state :bottom-right-side-contact))
                      (equal larm-motion-symbol :push-hold-up)
                      (equal rarm-motion-symbol :push-hold-up))
                 (and (or (equal floor-contact-state :bottom-left-side-contact)
                          (equal floor-contact-state :bottom-right-side-contact))
                      (equal larm-motion-symbol :support-hold-up)
                      (equal rarm-motion-symbol :support-hold-up)))
             :released)
            ((or (and (equal floor-contact-state :released)
                      (or (and (equal larm-motion-symbol :push-rotate-ccw-x45)
                               (equal rarm-motion-symbol :push-rotate-ccw-x45))
                          (and (equal larm-motion-symbol :push-rotate-cw-x45)
                               (equal rarm-motion-symbol :push-rotate-cw-x45))
                          (and (equal larm-motion-symbol :support-rotate-ccw-x45)
                               (equal rarm-motion-symbol :support-rotate-ccw-x45))
                          (and (equal larm-motion-symbol :support-rotate-cw-x45)
                               (equal rarm-motion-symbol :support-rotate-cw-x45))))
                 (and (equal floor-contact-state :released)
                      (equal larm-motion-symbol :push-hold-down)
                      (equal rarm-motion-symbol :push-hold-down)
                      (equal object-location-state :center))
                 (and (equal floor-contact-state :bottom-right-side-contact)
                      (equal larm-motion-symbol :left-move-push))
                 (and (equal floor-contact-state :bottom-left-side-contact)
                      (equal rarm-motion-symbol :right-move-push))
                 (and (equal floor-contact-state :bottom-left-side-contact)
                      (equal larm-motion-symbol :side-push-rotate-cw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-cw-x90)
                      (equal object-location-state :right))
                 (and (equal floor-contact-state :bottom-right-side-contact)
                      (equal larm-motion-symbol :side-push-rotate-ccw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-ccw-x90)
                      (equal object-location-state :left)))
             :bottom-contact)
            ((or (and (equal floor-contact-state :bottom-contact)
                      (equal larm-motion-symbol :left-move-push))
                 (and (equal floor-contact-state :released)
                      (equal larm-motion-symbol :support-hold-down)
                      (equal rarm-motion-symbol :support-hold-down)
                      (equal object-location-state :right))
                 (and (equal floor-contact-state :released)
                      (equal larm-motion-symbol :push-hold-down)
                      (equal rarm-motion-symbol :push-hold-down)
                      (equal object-location-state :right))
                 (and (equal floor-contact-state :bottom-contact)
                      (equal larm-motion-symbol :side-push-rotate-ccw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-ccw-x90)
                      (equal object-location-state :right)))
             :bottom-left-side-contact)
            ((or (and (equal floor-contact-state :bottom-contact)
                      (equal rarm-motion-symbol :right-move-push))
                 (and (equal floor-contact-state :released)
                      (equal larm-motion-symbol :support-hold-down)
                      (equal rarm-motion-symbol :support-hold-down)
                      (equal object-location-state :left))
                 (and (equal floor-contact-state :released)
                      (equal larm-motion-symbol :push-hold-down)
                      (equal rarm-motion-symbol :push-hold-down)
                      (equal object-location-state :left))
                 (and (equal floor-contact-state :bottom-contact)
                      (equal larm-motion-symbol :side-push-rotate-cw-x90)
                      (equal rarm-motion-symbol :side-push-rotate-cw-x90)
                      (equal object-location-state :left)))
             :bottom-right-side-contact)
            (t floor-contact-state)))
    (setq object-location-state
          (cond
            ((and (equal larm-motion-symbol :left-move-push)
                  (equal object-location-state :center))
             :right)
            ((and (equal rarm-motion-symbol :right-move-push)
                  (equal object-location-state :center))
             :left)
            ((or (and (equal larm-motion-symbol :left-move-push)
                      (equal object-location-state :left))
                 (and (equal rarm-motion-symbol :right-move-push)
                      (equal object-location-state :right)))
             :center)
            (t object-location-state)))
    (setq *object-contact-states*
          (list larm-contact-state rarm-contact-state floor-contact-state))
    (setq *object-location-state* object-location-state)))


(provide :common-planners "common-planners.l")
