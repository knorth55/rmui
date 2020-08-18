#!/usr/bin/env roseus

;; parameters

(defvar *device* (make-cube 100 100 100))
(defvar *world-frame-id* "world")
(defvar *device-frame-id* "rmui_link")


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
  ;; TODO: add if conditions
  (print contact-positions)
  )


(provide :rmui-planners "rmui-planners.l")
