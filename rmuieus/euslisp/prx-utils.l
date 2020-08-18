;; parameters

(defvar *prx-position-threshold* 0.1)


;; functions
(defvar sqrt1/5 (sqrt 0.2))
(defvar sqrt2/5 (* 2.0 sqrt1/5))
(defvar -sqrt1/5 (* -1.0 sqrt1/5))
(defvar -sqrt2/5 (* -1.0 sqrt2/5))


(defvar *prx-positions*
  (list
    ;; board lower
    (list :lower :front)
    (list :lower :left)
    (list :lower :back)
    (list :lower :right)
    (list :lower :center)
    ;; board front
    (list :front :left)
    (list :front :lower)
    (list :front :right)
    (list :front :upper)
    (list :front :center)
    ;; board left
    (list :left :back)
    (list :left :lower)
    (list :left :front)
    (list :left :upper)
    (list :left :center)
    ;; board back
    (list :back :right)
    (list :back :lower)
    (list :back :left)
    (list :back :upper)
    (list :back :center)
    ;; board right
    (list :right :front)
    (list :right :lower)
    (list :right :back)
    (list :right :upper)
    (list :right :center)
    ;; board upper
    (list :upper :front)
    (list :upper :right)
    (list :upper :back)
    (list :upper :left)
    (list :upper :center)))


(defvar *prx-vectors*
  (list
    ;; board lower
    (float-vector sqrt1/5 0.0 -sqrt2/5)
    (float-vector 0.0 sqrt1/5 -sqrt2/5)
    (float-vector -sqrt1/5 0.0 -sqrt2/5)
    (float-vector 0.0 -sqrt1/5 -sqrt2/5)
    (float-vector 0.0 0.0 -1.0)
    ;; board front
    (float-vector sqrt2/5 sqrt1/5 0.0)
    (float-vector sqrt2/5 0.0 -sqrt1/5)
    (float-vector sqrt2/5 -sqrt1/5 0.0)
    (float-vector sqrt2/5 0.0 sqrt1/5)
    (float-vector 1.0 0.0 0.0)
    ;; board left
    (float-vector -sqrt1/5 sqrt2/5 0.0)
    (float-vector 0.0 sqrt2/5 -sqrt1/5)
    (float-vector sqrt1/5 sqrt2/5 0.0)
    (float-vector 0.0 sqrt2/5 sqrt1/5)
    (float-vector 0.0 1.0 0.0)
    ;; board back
    (float-vector -sqrt2/5 -sqrt1/5 0.0)
    (float-vector -sqrt2/5 0.0 -sqrt1/5)
    (float-vector -sqrt2/5 sqrt1/5 0.0)
    (float-vector -sqrt2/5 0.0 sqrt1/5)
    (float-vector -1.0 0.0 0.0)
    ;; board right
    (float-vector sqrt1/5 -sqrt2/5 0.0)
    (float-vector 0.0 -sqrt2/5 -sqrt1/5)
    (float-vector -sqrt1/5 -sqrt2/5 0.0)
    (float-vector 0.0 -sqrt2/5 sqrt1/5)
    (float-vector 0.0 -1.0 0.0)
    ;; board upper
    (float-vector sqrt1/5 0.0 sqrt2/5)
    (float-vector 0.0 -sqrt1/5 sqrt2/5)
    (float-vector -sqrt1/5 0.0 sqrt2/5)
    (float-vector 0.0 sqrt1/5 sqrt2/5)
    (float-vector 0.0 0.0 1.0)))


(setq *cube-prx-vectors*
      (mapcar #'(lambda (prx-v) (copy-seq prx-v)) *prx-vectors*))


(defun update-cube-prx-vectors ()
  (let ((cube-rot (send *cube* :copy-rot)))
    (setq *cube-prx-vectors*
          (mapcar #'(lambda (prx-v) (let ((v (copy-seq prx-v))) (transform cube-rot v)))
                  *prx-vectors*))))


(defun get-prx-position (prx-v)
  (let*
    ((prx-v (scale (/ 1.0 (norm prx-v)) prx-v))
     (diff-vs (mapcar #'(lambda (v) (norm (v- prx-v v))) *prx-vectors*))
     (diff-p (mapcar #'(lambda (x) (< x *prx-position-threshold*)) diff-vs)))
    (when (every #'null diff-p) (return-from get-prx-position nil))
    (elt *prx-positions* (position t diff-p))))


(defun get-cube-prx-positions ()
  (update-cube-prx-vectors)
  (mapcar #'(lambda (prx-v) (get-prx-position prx-v)) *cube-prx-vectors*))


(provide :prx-utils "prx-utils.l")
