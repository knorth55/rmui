(setq *start-state* '(:init))
;; two boxes
(setq *goal-state* '(((:larm-contact-state :released)
                      (:rarm-contact-state :released)
                      (:floor-contact-state :released)
                      (:object-location-state :center)
                      (:object-rotation-state (:x0 :y0 :z90))
                      (:cube-contact-state (:never :bottom-contact))
                      (:left-wall-contact-state :released)
                      (:front-wall-contact-state :released))
                     ((:larm-contact-state :released)
                      (:rarm-contact-state :released)
                      (:floor-contact-state :bottom-contact)
                      (:object-location-state :center)
                      (:object-rotation-state (:x0 :y0 :z0))
                      (:cube-contact-state (:top-contact :never))
                      (:left-wall-contact-state :released)
                      (:front-wall-contact-state :released))))
;; three boxes
; (setq *goal-state* '(((:larm-contact-state :released)
;                       (:rarm-contact-state :released)
;                       (:floor-contact-state :bottom-contact)
;                       (:object-location-state :center)
;                       (:object-rotation-state (:x0 :y0 :z0))
;                       (:cube-contact-state (:never :top-contact :released)))
;                      ((:larm-contact-state :released)
;                       (:rarm-contact-state :released)
;                       (:floor-contact-state :released)
;                       (:object-location-state :center)
;                       (:object-rotation-state (:x0 :y0 :z0))
;                       (:cube-contact-state (:bottom-contact :never :top-contact)))
;                      ((:larm-contact-state :released)
;                       (:rarm-contact-state :released)
;                       (:floor-contact-state :released)
;                       (:object-location-state :center)
;                       (:object-rotation-state (:x0 :y0 :z0))
;                       (:cube-contact-state (:released :bottom-contact :never)))))