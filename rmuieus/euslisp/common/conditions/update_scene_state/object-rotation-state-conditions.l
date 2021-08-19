(((and (equal larm-motion-symbol :push-rotate-ccw-z45)
       (equal rarm-motion-symbol :push-rotate-ccw-z45))
  (list prev-object-x-rotation-symbol
        prev-object-y-rotation-symbol
        (rotate-rotation-symbol prev-object-z-rotation-symbol 45)))
 ((and (equal larm-motion-symbol :push-rotate-cw-z45)
       (equal rarm-motion-symbol :push-rotate-cw-z45))
  (list prev-object-x-rotation-symbol
        prev-object-y-rotation-symbol
        (rotate-rotation-symbol prev-object-z-rotation-symbol -45)))
 ((or (and (equal larm-motion-symbol :push-rotate-ccw-x45)
           (equal rarm-motion-symbol :push-rotate-ccw-x45))
      (and (equal larm-motion-symbol :support-rotate-ccw-x45)
           (equal rarm-motion-symbol :support-rotate-ccw-x45)))
  (list (rotate-rotation-symbol prev-object-x-rotation-symbol 45)
        prev-object-y-rotation-symbol
        prev-object-z-rotation-symbol))
 ((or (and (equal larm-motion-symbol :push-rotate-cw-x45)
           (equal rarm-motion-symbol :push-rotate-cw-x45))
      (and (equal larm-motion-symbol :support-rotate-cw-x45)
           (equal rarm-motion-symbol :support-rotate-cw-x45)))
  (list (rotate-rotation-symbol prev-object-x-rotation-symbol -45)
        prev-object-y-rotation-symbol
        prev-object-z-rotation-symbol))
 ((or (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-ccw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :side-push-rotate-ccw-x90)))
  (list (rotate-rotation-symbol prev-object-x-rotation-symbol 90)
        prev-object-y-rotation-symbol
        prev-object-z-rotation-symbol))
 ((or (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-cw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
           (equal rarm-motion-symbol :side-push-rotate-cw-x90)))
  (list (rotate-rotation-symbol prev-object-x-rotation-symbol -90)
        prev-object-y-rotation-symbol
        prev-object-z-rotation-symbol))
 )
