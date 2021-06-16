#!/usr/bin/env roseus

(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")


;; init

(defun planner-init (&key (cube-pos-y 0))
  (motion-init :cube-pos-y cube-pos-y))


;; motion planners

(defun dualarm-push-hold-release (&key (move-robot t) (use-torso 0.001))
  (let* ((released (hold-release-motion :move-robot nil :use-torso use-torso))
         (avs (car released))
         (tms (cadr released)))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence avs tms nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-release)))


(defun dualarm-push-hold-up
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (move-robot t)
        (use-torso 0.001))
  (let* ((holded
           (push-hold-motion :push-axis push-axis
                             :push-position push-position
                             :approach-direction approach-direction
                             :top-arm nil
                             :front-arm front-arm
                             :move-robot nil
                             :use-torso use-torso))
         (lifted-up
           (when holded (hold-lift-up-motion :move-robot nil
                                             :use-torso use-torso))))
    (when lifted-up
      (when move-robot
        (send *ri* :angle-vector-sequence (car holded) (cadr holded) nil 0)
        (send *ri* :wait-interpolation)
        (assoc-cube :arms)
        (send *ri* :angle-vector-sequence (car lifted-up) (cadr lifted-up) nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-up)))


(defun dualarm-push-hold-down (&key (move-robot t) (use-torso 0.001))
  (let* ((lifted-down (hold-lift-down-motion :move-robot nil :use-torso use-torso))
         (released (when lifted-down (hold-release-motion :move-robot nil
                                                          :use-torso use-torso))))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence (car lifted-down) (cadr lifted-down) nil 0)
        (send *ri* :wait-interpolation)
        (dissoc-cube :arms)
        (send *ri* :angle-vector-sequence (car released) (cadr released) nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-down)))


(defun dualarm-push-rotate
  (&key (rotate-angle pi/2) (rotate-axis :x)
        (front-arm :larm) (hold-type :light)
        (move-robot t) (use-torso 0.001))
  (let* ((holded
           (push-hold-motion :push-axis (if (eq rotate-axis :y) :x :y)
                             :push-position :center
                             :approach-direction
                             (cond ((eq rotate-axis :x) :back)
                                   ((eq rotate-axis :y) :side)
                                   ((eq rotate-axis :z) :top))
                             :top-arm nil
                             :front-arm front-arm
                             :hold-type hold-type
                             :move-robot nil
                             :use-torso use-torso))
         (rotated
           (when holded (push-rotate-motion :rotate-angle rotate-angle
                                            :rotate-axis rotate-axis
                                            :move-robot nil
                                            :use-torso use-torso)))
         (released
           (when rotated (hold-release-motion :move-robot nil
                                              :use-torso use-torso))))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence (car holded) (cadr holded) nil 0)
        (send *ri* :wait-interpolation)
        (assoc-cube :arms)
        (send *ri* :angle-vector-sequence (car rotated) (cadr rotated) nil 0)
        (send *ri* :wait-interpolation)
        (dissoc-cube :arms)
        (send *ri* :angle-vector-sequence (car released) (cadr released) nil 0)
        (send *ri* :wait-interpolation))
      :push-rotate)))


(defun dualarm-support-rotate
  (&key (rotate-angle pi/2) (rotate-axis :x)
        (top-arm :larm) (front-arm nil) (back-arm nil) (hold-type :light)
        (move-robot t) (use-torso 0.001))
  (let* ((approach-direction (cond ((eq rotate-axis :x) :back)
                                   ((eq rotate-axis :y) :side)))
         (top-arm (cond ((eq rotate-axis :x)
                           (if (> rotate-angle 0) :rarm :larm))
                          (t top-arm)))
         (front-arm (cond ((eq rotate-axis :y)
                           (if (eq top-arm :larm)
                             (if (> rotate-angle 0) nil :rarm)
                             (if (> rotate-angle 0) nil :larm)))
                          (t front-arm)))
         (back-arm (cond ((eq rotate-axis :y)
                          (if (eq top-arm :larm)
                            (if (> rotate-angle 0) :rarm nil)
                            (if (> rotate-angle 0) :larm nil)))
                          (t back-arm)))
         (holded
           (support-hold-motion :support-axis rotate-axis
                                :push-position :center
                                :approach-direction approach-direction
                                :top-arm top-arm
                                :front-arm front-arm
                                :back-arm back-arm
                                :hold-type hold-type
                                :move-robot nil
                                :switch nil
                                :use-torso use-torso))
         (rotated
           (when holded (support-rotate-motion :rotate-angle rotate-angle
                                               :rotate-axis rotate-axis
                                               :move-robot nil
                                               :use-torso use-torso)))
         (released
           (when rotated (hold-release-motion :move-robot nil
                                              :use-torso use-torso)))
         (avs (when released (append (car holded) (car rotated) (car released))))
         (tms (when released (append (cadr holded) (cadr rotated) (cadr released)))))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence (car holded) (cadr holded) nil 0)
        (send *ri* :wait-interpolation)
        (assoc-cube :arms)
        (send *ri* :angle-vector-sequence (car rotated) (cadr rotated) nil 0)
        (send *ri* :wait-interpolation)
        (dissoc-cube :arms)
        (send *ri* :angle-vector-sequence (car released) (cadr released) nil 0)
        (send *ri* :wait-interpolation))
      :support-rotate)))


(defun dualarm-switch-rotate
  (&key (rotate-angle pi/2) (rotate-axis :x)
        (top-arm :larm) (front-arm nil) (back-arm nil) (hold-type :tight)
        (move-robot t) (use-torso 0.001))
  (when (eq rotate-axis :z)
    (when (dualarm-push-rotate :rotate-angle rotate-angle :rotate-axis rotate-axis
                               :front-arm front-arm :hold-type hold-type
                               :move-robot move-robot :use-torso use-torso)
      (return-from dualarm-switch-rotate :switch-rotate)))
  (let* ((cube-height (get-cube-height))
         (cube-width (get-cube-width))
         (cube-depth (get-cube-depth))
         (push-rotate-angle
           (get-switch-rotate-angle :rotate-angle rotate-angle :rotate-axis rotate-axis))
         (support-rotate-angle (- rotate-angle push-rotate-angle))
         (approach-direction (cond ((eq rotate-axis :x) :back)
                                   ((eq rotate-axis :y) :side)))
         (top-arm (cond ((eq rotate-axis :x)
                           (if (> rotate-angle 0) :rarm :larm))
                          (t top-arm)))
         (push-front-arm (cond ((eq rotate-axis :y)
                                (if (> rotate-angle 0)
                                  (if (eq top-arm :larm) :larm :rarm)
                                  (if (eq top-arm :larm) :rarm :larm)))
                               (t front-arm)))
         (support-front-arm (cond ((eq rotate-axis :y)
                                   (if (eq top-arm :larm)
                                     (if (> rotate-angle 0) nil :rarm)
                                     (if (> rotate-angle 0) nil :larm)))
                                  (t front-arm)))
         (back-arm (cond ((eq rotate-axis :y)
                          (if (eq top-arm :larm)
                            (if (> rotate-angle 0) :rarm nil)
                            (if (> rotate-angle 0) :larm nil)))
                          (t back-arm)))
         (push-holded
           (push-hold-motion :push-axis (if (eq rotate-axis :y) :x :y)
                             :push-position :center
                             :approach-direction approach-direction
                             :top-arm nil
                             :front-arm push-front-arm
                             :hold-type hold-type
                             :move-robot nil
                             :use-torso use-torso
                             :cube-height cube-height
                             :cube-width cube-width
                             :cube-depth cube-depth))
         (push-rotated
           (when push-holded (push-rotate-motion :rotate-angle push-rotate-angle
                                                 :rotate-axis rotate-axis
                                                 :move-robot nil
                                                 :use-torso use-torso
                                                 :cube-height cube-height
                                                 :cube-width cube-width
                                                 :cube-depth cube-depth)))
         (push-released
           (when push-rotated (hold-release-motion :arm top-arm
                                                   :move-robot nil
                                                   :use-torso use-torso)))
         (support-holded
           (when push-released
             (support-hold-motion :support-axis rotate-axis
                                  :push-position :center
                                  :approach-direction approach-direction
                                  :approach-arm top-arm
                                  :top-arm top-arm
                                  :front-arm support-front-arm
                                  :back-arm back-arm
                                  :hold-type hold-type
                                  :move-robot nil
                                  :switch t
                                  :use-torso use-torso
                                  :cube-rpy
                                  (cond
                                    ((eq rotate-axis :x)
                                     (float-vector 0 0 push-rotate-angle))
                                    ((eq rotate-axis :y)
                                     (float-vector 0 push-rotate-angle 0))
                                    (t (float-vector 0 0 0)))
                                  :cube-height cube-height
                                  :cube-width cube-width
                                  :cube-depth cube-depth)))
         (support-rotated
           (when support-holded
             (support-rotate-motion :rotate-angle support-rotate-angle
                                    :initial-rotate-angle push-rotate-angle
                                    :rotate-axis rotate-axis
                                    :move-robot nil
                                    :use-torso use-torso
                                    :cube-height cube-height
                                    :cube-width cube-width
                                    :cube-depth cube-depth)))
         (support-released
           (when support-rotated (hold-release-motion :arm :arms
                                                      :move-robot nil
                                                      :use-torso use-torso)))
         (avs (when support-released
                (append (car push-holded) (car push-rotated) (car push-released)
                        (car support-holded) (car support-rotated) (car support-released))))
         (tms (when support-released
                (append (cadr push-holded) (cadr push-rotated) (cadr push-released)
                        (cadr support-holded) (cadr support-rotated) (cadr support-released)))))
    (when support-released
      (when move-robot
        (send *ri* :angle-vector-sequence (car push-holded) (cadr push-holded) nil 0)
        (send *ri* :wait-interpolation)
        (assoc-cube :arms)
        (send *ri* :angle-vector-sequence (car push-rotated) (cadr push-rotated) nil 0)
        (send *ri* :wait-interpolation)
        (dissoc-cube :arms)
        (send *ri* :angle-vector-sequence
              (append (car push-released) (car support-holded))
              (append (cadr push-released) (cadr support-holded))
              nil 0)
        (send *ri* :wait-interpolation)
        (assoc-cube :arms)
        (send *ri* :angle-vector-sequence (car support-rotated) (cadr support-rotated) nil 0)
        (send *ri* :wait-interpolation)
        (dissoc-cube :arms)
        (send *ri* :angle-vector-sequence (car support-released) (cadr support-released) nil 0)
        (send *ri* :wait-interpolation))
      :switch-rotate)))


(provide :pr2-planners "pr2-planners.l")
