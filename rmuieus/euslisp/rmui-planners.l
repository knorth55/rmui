#!/usr/bin/env roseus

;; parameters

(defvar *device* (make-cube 100 100 100))
(defvar *world-frame-id* "world")
(defvar *device-frame-id* "rmui_link")
(defvar *motion-symbol-length* 20)


;; functions

(setq *device-state* :released)
(setq *motion-symbols* nil)

(defun get-contact-positions (proximity)
  (let ((prx-positions (get-device-prx-positions))
        (prx-index 0)
        (contact-positions nil))
    (dolist (prx proximity)
      (when (> prx *prx-threshold*)
        (setq contact-positions
              (append contact-positions
                      (list (elt prx-positions prx-index)))))
      (setq prx-index (+ prx-index 1)))
    contact-positions))


(defun get-motion-symbol (contact-positions)
  (let ((contact-surfaces (mapcar #'car contact-positions)))
    ;; TODO: add rotate conditions
    (cond
      ((and (or (eq *device-state* :released)
                (eq *device-state* :rarm-right-pushed))
            (find :left contact-surfaces))
       :larm-left-push)
      ((and (or (eq *device-state* :released)
                (eq *device-state* :rarm-back-pushed))
            (find :front contact-surfaces))
       :larm-front-push)
      ((and (or (eq *device-state* :released)
                (eq *device-state* :rarm-front-pushed))
            (find :back contact-surfaces))
       :larm-back-push)
      ((and (or (eq *device-state* :released)
                (eq *device-state* :larm-left-pushed))
            (find :right contact-surfaces))
       :rarm-right-push)
      ((and (or (eq *device-state* :released)
                (eq *device-state* :larm-back-pushed))
            (find :front contact-surfaces))
       :rarm-front-push)
      ((and (or (eq *device-state* :released)
                (eq *device-state* :larm-front-pushed))
            (find :back contact-surfaces))
       :rarm-back-push)
      ((and (or (eq *device-state* :push-x-larm-front-holded)
                (eq *device-state* :push-x-rarm-front-holded)
                (eq *device-state* :push-y-holded))
            (null (find :lower contact-surfaces)))
       :push-hold-up)
      ((and (or (eq *device-state* :push-x-larm-front-holded-up)
                (eq *device-state* :push-x-rarm-front-holded-up)
                (eq *device-state* :push-y-holded-up))
            (find :lower contact-surfaces))
       :push-hold-down)
      ((or (and (eq *device-state* :push-x-larm-front-holded)
                (null (find :front contact-surfaces)))
           (and (eq *device-state* :push-x-rarm-front-holded)
                (null (find :back contact-surfaces)))
           (and (eq *device-state* :push-y-holded)
                (null (find :left contact-surfaces)))
           (and (eq *device-state* :larm-left-pushed)
                (null (find :left contact-surfaces)))
           (and (eq *device-state* :larm-front-pushed)
                (null (find :front contact-surfaces)))
           (and (eq *device-state* :larm-back-pushed)
                (null (find :back contact-surfaces))))
       :larm-push-release)
      ((or (and (eq *device-state* :push-x-larm-front-holded)
                (null (find :back contact-surfaces)))
           (and (eq *device-state* :push-x-rarm-front-holded)
                (null (find :front contact-surfaces)))
           (and (eq *device-state* :push-y-holded)
                (null (find :right contact-surfaces)))
           (and (eq *device-state* :rarm-right-pushed)
                (null (find :right contact-surfaces)))
           (and (eq *device-state* :rarm-front-pushed)
                (null (find :front contact-surfaces)))
           (and (eq *device-state* :rarm-back-pushed)
                (null (find :back contact-surfaces))))
       :rarm-push-release)
      ((null contact-positions) nil)
      (t nil))))


(defun execute-motion-symbol (motion-symbol &key (move-robot t) (use-torso 0.001))
  (unless motion-symbol (return-from execute-motion-symbol nil))
  (setq *motion-symbols*
        (append
          (if (>= (length *motion-symbols*) *motion-symbol-length*)
            (cdr *motion-symbols*) *motion-symbols*)
          (list motion-symbol)))
  (let ((execute-p (and (>= (length *motion-symbols*) *motion-symbol-length*)
                        (every #'(lambda (x) (eq x motion-symbol)) *motion-symbols*))))
    (when execute-p
      (cond
        ((eq motion-symbol :larm-left-push)
         (push-motion :larm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :y)
                      :push-positions (list :center)
                      :push-directions (list :left)
                      :approach-directions (list :back)
                      :approach-arm :larm))
        ((eq motion-symbol :larm-front-push)
         (push-motion :larm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :x)
                      :push-positions (list :center)
                      :push-directions (list :front)
                      :approach-directions (list :left)
                      :approach-arm :larm))
        ((eq motion-symbol :larm-back-push)
         (push-motion :larm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :x)
                      :push-positions (list :center)
                      :push-directions (list :back)
                      :approach-directions (list :left)
                      :approach-arm :larm))
        ((eq motion-symbol :rarm-right-push)
         (push-motion :rarm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :y)
                      :push-positions (list :center)
                      :push-directions (list :right)
                      :approach-directions (list :back)
                      :approach-arm :rarm))
        ((eq motion-symbol :rarm-front-push)
         (push-motion :rarm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :x)
                      :push-positions (list :center)
                      :push-directions (list :front)
                      :approach-directions (list :right)
                      :approach-arm :rarm))
        ((eq motion-symbol :rarm-back-push)
         (push-motion :rarm :move-robot move-robot :use-torso use-torso
                      :push-axes (list :x)
                      :push-positions (list :center)
                      :push-directions (list :back)
                      :approach-directions (list :right)
                      :approach-arm :rarm))
        ((eq motion-symbol :push-hold-up)
         (hold-lift-up-motion :move-robot move-robot :use-torso use-torso))
        ((eq motion-symbol :push-hold-down)
         (hold-lift-down-motion :move-robot move-robot :use-torso use-torso))
        ((eq motion-symbol :larm-push-release)
         (push-release-motion :larm :move-robot move-robot :use-torso use-torso))
        ((eq motion-symbol :rarm-push-release)
         (push-release-motion :rarm :move-robot move-robot :use-torso use-torso))
        (t (ros::ros-error (format nil "invalid motion-symbol: ~A" motion-symbol))))
      (setq *device-state*
            (cond
              ;; released -> larm pushed
              ((and (eq motion-symbol :larm-left-push)
                    (eq *device-state* :released))
               :larm-left-pushed)
              ((and (eq motion-symbol :larm-front-push)
                    (eq *device-state* :released))
               :larm-front-pushed)
              ((and (eq motion-symbol :larm-back-push)
                    (eq *device-state* :released))
               :larm-back-pushed)
              ;; released -> rarm pushed
              ((and (eq motion-symbol :rarm-right-push)
                    (eq *device-state* :released))
               :rarm-right-pushed)
              ((and (eq motion-symbol :rarm-front-push)
                    (eq *device-state* :released))
               :rarm-front-pushed)
              ((and (eq motion-symbol :rarm-back-push)
                    (eq *device-state* :released))
               :rarm-back-pushed)
              ;; rarm pushed -> push holded
              ((and (eq motion-symbol :larm-left-push)
                    (eq *device-state* :rarm-right-pushed))
               :push-y-holded)
              ((and (eq motion-symbol :larm-front-push)
                    (eq *device-state* :rarm-back-pushed))
               :push-x-larm-front-holded)
              ((and (eq motion-symbol :larm-back-push)
                    (eq *device-state* :rarm-front-pushed))
               :push-x-rarm-front-holded)
              ;; larm pushed -> push holded
              ((and (eq motion-symbol :rarm-right-push)
                    (eq *device-state* :larm-left-pushed))
               :push-y-holded)
              ((and (eq motion-symbol :rarm-front-push)
                    (eq *device-state* :larm-back-pushed))
               :push-x-rarm-front-holded)
              ((and (eq motion-symbol :rarm-back-push)
                    (eq *device-state* :larm-front-pushed))
               :push-x-larm-front-holded)
              ;; holded -> holed up
              ((and (eq motion-symbol :push-hold-up)
                    (eq *device-state* :push-y-holded))
               :push-y-holded-up)
              ((and (eq motion-symbol :push-hold-up)
                    (eq *device-state* :push-x-larm-front-holded))
               :push-x-larm-front-holded-up)
              ((and (eq motion-symbol :push-hold-up)
                    (eq *device-state* :push-x-rarm-front-holded))
               :push-x-rarm-front-holded-up)
              ;; holded up -> holded
              ((and (eq motion-symbol :push-hold-down)
                    (eq *device-state* :push-y-holded-up))
               :push-y-holded)
              ((and (eq motion-symbol :push-hold-down)
                    (eq *device-state* :push-x-larm-front-holded-up))
               :push-x-larm-front-holded)
              ((and (eq motion-symbol :push-hold-down)
                    (eq *device-state* :push-x-rarm-front-holded-up))
               :push-x-rarm-front-holded)
              ;; holded -> rarm pushed
              ((and (eq motion-symbol :larm-push-release)
                    (eq *device-state* :push-y-holded))
               :rarm-right-pushed)
              ((and (eq motion-symbol :larm-push-release)
                    (eq *device-state* :push-x-larm-front-holded))
               :rarm-back-pushed)
              ((and (eq motion-symbol :larm-push-release)
                    (eq *device-state* :push-x-rarm-front-holded))
               :rarm-front-pushed)
              ;; holded -> larm pushed
              ((and (eq motion-symbol :rarm-push-release)
                    (eq *device-state* :push-y-holded))
               :larm-left-pushed)
              ((and (eq motion-symbol :rarm-push-release)
                    (eq *device-state* :push-x-larm-front-holded))
               :larm-front-pushed)
              ((and (eq motion-symbol :rarm-push-release)
                    (eq *device-state* :push-x-rarm-front-holded))
               :larm-back-pushed)
              ;; larm pushed -> released
              ((and (eq motion-symbol :larm-push-release)
                    (or (eq *device-state* :larm-left-pushed)
                        (eq *device-state* :larm-front-pushed)
                        (eq *device-state* :larm-back-pushed)))
               :released)
              ;; rarm pushed -> released
              ((and (eq motion-symbol :rarm-push-release)
                    (or (eq *device-state* :rarm-right-pushed)
                        (eq *device-state* :rarm-front-pushed)
                        (eq *device-state* :rarm-back-pushed)))
               :released)
              (t nil))))
    execute-p))


(provide :rmui-planners "rmui-planners.l")