(setq *start-state* '(:init))
(setq *goal-state* '(((:larm-contact-state :released)
                      (:rarm-contact-state :released)
                      (:floor-contact-state :bottom-contact)
                      (:object-location-state :center)
                      (:object-rotation-state (:x0 :y0 :z90))
                      (:cube-contact-state (list :never :released :released))
                      (:left-wall-contact-state :released)
                      (:front-wall-contact-state :released)
                      )))
