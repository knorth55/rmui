(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'rmui))
(send *domain* :requirements '(:typing :equality :conditional-effects :quantified-preconditions))
(send *domain* :types '(object spot arm lside rside))
(send *domain* :constants '(CUBE - object
                            LARM RARM - arm
                            TABLE - spot
                            LEFT LEFT-FRONT LEFT-BACK - lside
                            RIGHT RIGHT-FRONT RIGHT-BACK - rside
                            LEFT LEFT-FRONT LEFT-BACK RIGHT RIGHT-FRONT RIGHT-BACK - side
                            Z45 Z-45 - rotate-zangle
                            Z0 Z45 Z90 Z135 Z180 Z-45 Z-90 Z-135 - zangle
                            ))
(send *domain* :predicates '((ON ?obj - object ?spot - spot)
                             (AT ?spot - spot)
                             (CONTACT ?obj - object ?arm - arm ?side - side)
                             (ZROTATE ?obj - object ?zangle - zangle)
                             (HOLDUP ?obj - object)
                             ))

(provide :rmui-pddl-domain "rmui-pddl-domain.l")