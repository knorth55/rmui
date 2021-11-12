(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


;; parameters
(defvar *cube-id* "cube")
;; (defvar *cube* (make-cube 350 350 300 :name *cube-id*)) ;; cardboard box
(defvar *cube* (make-cube 270 420 220 :name *cube-id*))  ;; ctb
;; (defvar *cube* (make-cube 270 370 500 :name *cube-id*))  ;; trash box
;; (defvar *cube* (make-cube 150 280 320 :name *cube-id*))  ;; shredder
(defvar *desk-id* "room73b2-askul-1200x700-desk-0")
(defvar *desk* (room73b2-askul-1200x700-desk-0))

(provide :common-scenes "common-scenes.l")
