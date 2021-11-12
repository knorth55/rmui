(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


;; parameters
(defvar *box-id* "box")
;; (defvar *box* (make-cube 350 350 300 :name *box-id*)) ;; cardboard box
(defvar *box* (make-cube 270 420 220 :name *box-id*))  ;; ctb
;; (defvar *box* (make-cube 270 370 500 :name *box-id*))  ;; trash box
;; (defvar *box* (make-cube 150 280 320 :name *box-id*))  ;; shredder
(defvar *table-id* "room73b2-askul-1200x700-desk-0")
(defvar *table* (room73b2-askul-1200x700-desk-0))

(provide :common-scenes "common-scenes.l")
