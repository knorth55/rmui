;; functions
(defvar sqrt1/5 (sqrt 0.2))
(defvar sqrt4/5 (* 2.0 sqrt1/5))
(defvar -sqrt1/5 (* -1.0 sqrt1/5))
(defvar -sqrt4/5 (* -1.0 sqrt4/5))


(defvar *prx-initial-positions*
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


(defvar *prx-x45-positions*
  (list
    ;; board bottom
    (list :bottom-left :front)
    (list :bottom-left :left)
    (list :bottom-left :back)
    (list :bottom-left :right)
    (list :bottom-left :center)
    ;; board front
    (list :front :top-left)
    (list :front :bottom-left)
    (list :front :bottom-right)
    (list :front :top-right)
    (list :front :center)
    ;; board left
    (list :top-left :back)
    (list :top-left :bottom)
    (list :top-left :front)
    (list :top-left :top)
    (list :top-left :center)
    ;; board back
    (list :back :bottom-right)
    (list :back :bottom-left)
    (list :back :top-left)
    (list :back :top-right)
    (list :back :center)
    ;; board right
    (list :bottom-right :front)
    (list :bottom-right :bottom)
    (list :bottom-right :back)
    (list :bottom-right :top)
    (list :bottom-right :center)
    ;; board top
    (list :top-right :front)
    (list :top-right :right)
    (list :top-right :back)
    (list :top-right :left)
    (list :top-right :center)))


(defvar *prx-y45-positions*
  (list
    ;; board bottom
    (list :bottom-back :front)
    (list :bottom-back :left)
    (list :bottom-back :back)
    (list :bottom-back :right)
    (list :bottom-back :center)
    ;; board front
    (list :bottom-front :left)
    (list :bottom-front :bottom)
    (list :bottom-front :right)
    (list :bottom-front :top)
    (list :bottom-front :center)
    ;; board left
    (list :left :top-back)
    (list :left :bottom-back)
    (list :left :bottom-front)
    (list :left :top-front)
    (list :left :center)
    ;; board back
    (list :top-back :right)
    (list :top-back :bottom)
    (list :top-back :left)
    (list :top-back :top)
    (list :top-back :center)
    ;; board right
    (list :right :bottom-front)
    (list :right :bottom-back)
    (list :right :top-back)
    (list :right :top-front)
    (list :right :center)
    ;; board top
    (list :top-front :front)
    (list :top-front :right)
    (list :top-front :back)
    (list :top-front :left)
    (list :top-front :center)))


(defvar *prx-z45-positions*
  (list
    ;; board bottom
    (list :bottom :front-left)
    (list :bottom :back-left)
    (list :bottom :back-right)
    (list :bottom :front-right)
    (list :bottom :center)
    ;; board front
    (list :front-left :left)
    (list :front-left :bottom)
    (list :front-left :right)
    (list :front-left :top)
    (list :front-left :center)
    ;; board left
    (list :back-left :back)
    (list :back-left :bottom)
    (list :back-left :front)
    (list :back-left :top)
    (list :back-left :center)
    ;; board back
    (list :back-right :right)
    (list :back-right :bottom)
    (list :back-right :left)
    (list :back-right :top)
    (list :back-right :center)
    ;; board right
    (list :front-right :front)
    (list :front-right :bottom)
    (list :front-right :back)
    (list :front-right :top)
    (list :front-right :center)
    ;; board top
    (list :top :front-left)
    (list :top :front-right)
    (list :top :back-right)
    (list :top :back-left)
    (list :top :center)))


(defvar *prx-initial-vectors*
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


(defvar *prx-x45-vectors*
  (mapcar #'(lambda (v) (transform (rotation-matrix (/ pi 4.0) :x) v))
          *prx-initial-vectors*))


(defvar *prx-y45-vectors*
  (mapcar #'(lambda (v) (transform (rotation-matrix (/ pi 4.0) :y) v))
          *prx-initial-vectors*))


(defvar *prx-z45-vectors*
  (mapcar #'(lambda (v) (transform (rotation-matrix (/ pi 4.0) :z) v))
          *prx-initial-vectors*))


(defun copy-prx-vectors (prx-vectors)
  (mapcar #'(lambda (prx-v) (copy-seq prx-v)) prx-vectors))


(provide :prx-utils "prx-utils.l")
