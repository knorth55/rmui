;; functions
(defvar sqrt1/5 (sqrt 0.2))
(defvar sqrt4/5 (* 2.0 sqrt1/5))
(defvar -sqrt1/5 (* -1.0 sqrt1/5))
(defvar -sqrt4/5 (* -1.0 sqrt4/5))


(defvar *prx-positions*
  (list
    ;; board bottom
    (list :bottom :front)
    (list :bottom :left)
    (list :bottom :back)
    (list :bottom :right)
    (list :bottom :center)
    ;; board front
    (list :front :left)
    (list :front :bottom)
    (list :front :right)
    (list :front :top)
    (list :front :center)
    ;; board left
    (list :left :back)
    (list :left :bottom)
    (list :left :front)
    (list :left :top)
    (list :left :center)
    ;; board back
    (list :back :right)
    (list :back :bottom)
    (list :back :left)
    (list :back :top)
    (list :back :center)
    ;; board right
    (list :right :front)
    (list :right :bottom)
    (list :right :back)
    (list :right :top)
    (list :right :center)
    ;; board top
    (list :top :front)
    (list :top :right)
    (list :top :back)
    (list :top :left)
    (list :top :center)))


(defvar *prx-vectors*
  (list
    ;; board bottom
    (float-vector sqrt1/5 0.0 -sqrt4/5)
    (float-vector 0.0 sqrt1/5 -sqrt4/5)
    (float-vector -sqrt1/5 0.0 -sqrt4/5)
    (float-vector 0.0 -sqrt1/5 -sqrt4/5)
    (float-vector 0.0 0.0 -1.0)
    ;; board front
    (float-vector sqrt4/5 sqrt1/5 0.0)
    (float-vector sqrt4/5 0.0 -sqrt1/5)
    (float-vector sqrt4/5 -sqrt1/5 0.0)
    (float-vector sqrt4/5 0.0 sqrt1/5)
    (float-vector 1.0 0.0 0.0)
    ;; board left
    (float-vector -sqrt1/5 sqrt4/5 0.0)
    (float-vector 0.0 sqrt4/5 -sqrt1/5)
    (float-vector sqrt1/5 sqrt4/5 0.0)
    (float-vector 0.0 sqrt4/5 sqrt1/5)
    (float-vector 0.0 1.0 0.0)
    ;; board back
    (float-vector -sqrt4/5 -sqrt1/5 0.0)
    (float-vector -sqrt4/5 0.0 -sqrt1/5)
    (float-vector -sqrt4/5 sqrt1/5 0.0)
    (float-vector -sqrt4/5 0.0 sqrt1/5)
    (float-vector -1.0 0.0 0.0)
    ;; board right
    (float-vector sqrt1/5 -sqrt4/5 0.0)
    (float-vector 0.0 -sqrt4/5 -sqrt1/5)
    (float-vector -sqrt1/5 -sqrt4/5 0.0)
    (float-vector 0.0 -sqrt4/5 sqrt1/5)
    (float-vector 0.0 -1.0 0.0)
    ;; board top
    (float-vector sqrt1/5 0.0 sqrt4/5)
    (float-vector 0.0 -sqrt1/5 sqrt4/5)
    (float-vector -sqrt1/5 0.0 sqrt4/5)
    (float-vector 0.0 sqrt1/5 sqrt4/5)
    (float-vector 0.0 0.0 1.0)))


(defvar *cube-prx-vectors*
  (mapcar #'(lambda (prx-v) (copy-seq prx-v)) *prx-vectors*))


(defvar *device-prx-vectors*
  (mapcar #'(lambda (prx-v) (copy-seq prx-v)) *prx-vectors*))


(defun get-prx-position (prx-v prx-position-threshold)
  (let*
    ((prx-v (scale (/ 1.0 (norm prx-v)) prx-v))
     (diff-vs (mapcar #'(lambda (v) (norm (v- prx-v v))) *prx-vectors*))
     (diff-p (mapcar #'(lambda (x) (< x prx-position-threshold)) diff-vs)))
    (when (every #'null diff-p) (return-from get-prx-position nil))
    (elt *prx-positions* (position t diff-p))))


(defun update-cube-prx-vectors ()
  (unless (boundp '*cube*) (return-from update-cube-prx-vectors nil))
  (let ((cube-rot (send *cube* :copy-rot)))
    (setq *cube-prx-vectors*
          (mapcar #'(lambda (prx-v) (let ((v (copy-seq prx-v))) (transform cube-rot v)))
                  *prx-vectors*))
    t))


(defun get-cube-prx-positions (prx-position-threshold)
  (when (update-cube-prx-vectors)
    (mapcar #'(lambda (prx-v)
                (get-prx-position prx-v prx-position-threshold))
            *cube-prx-vectors*)))

(defun update-device-prx-vectors ()
  (unless (boundp '*device*) (return-from update-device-prx-vectors nil))
  (let ((device-rot (send *device* :copy-rot)))
    (setq *device-prx-vectors*
          (mapcar #'(lambda (prx-v) (let ((v (copy-seq prx-v))) (transform device-rot v)))
                  *prx-vectors*))
    t))


(defun get-device-prx-positions (prx-position-threshold)
  (when (update-device-prx-vectors)
    (mapcar #'(lambda (prx-v)
                (get-prx-position prx-v prx-position-threshold))
            *device-prx-vectors*)))


(defun get-contact-positions (proximity prx-threshold prx-position-threshold)
  (let ((prx-positions (get-device-prx-positions prx-position-threshold))
        (prx-index 0)
        (contact-positions nil))
    (dolist (prx proximity)
      (when (> prx prx-threshold)
        (setq contact-positions
              (append contact-positions
                      (list (elt prx-positions prx-index)))))
      (setq prx-index (+ prx-index 1)))
    contact-positions))


(provide :prx-utils "prx-utils.l")
