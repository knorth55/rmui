(((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :left-contact))
       (or (equal floor-contact-state :bottom-contact)
           (equal floor-contact-state :bottom-left-side-contact)
           (equal floor-contact-state :bottom-right-side-contact))
       (find :left contact-surface)
       (find :bottom contact-surface))
  :left-push)
 ;; TODO: front-push for rarm
 ; ((and (equal larm-contact-state :released)
 ;       (null (equal rarm-contact-state :front-contact))
 ;       (or (equal floor-contact-state :bottom-contact)
 ;           (equal floor-contact-state :bottom-left-side-contact)
 ;           (equal floor-contact-state :bottom-right-side-contact))
 ;       (find :front contact-surface)
 ;       (find :bottom contact-surface))
 ;  :front-push)
 ((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :back-contact))
       (or (equal floor-contact-state :bottom-contact)
           (equal floor-contact-state :bottom-left-side-contact)
           (equal floor-contact-state :bottom-right-side-contact))
       (find :back contact-surface)
       (find :bottom contact-surface))
  :back-push)
 ;; TODO: top-push for rarm
 ; ((and (equal larm-contact-state :released)
 ;       (null (equal rarm-contact-state :top-contact))
 ;       (null (equal rarm-contact-state :bottom-right-side-contact))
 ;       (or (equal floor-contact-state :bottom-contact)
 ;           (equal floor-contact-state :bottom-left-side-contact)
 ;           (equal floor-contact-state :bottom-right-side-contact))
 ;       (find :top contact-surface)
 ;       (find :bottom contact-surface))
 ;  :top-push)
 ((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :top-left-contact))
       (equal floor-contact-state :released)
       (find :top-left contact-surface)
       (null (find :bottom contact-surface)))
  :top-left-push)
 ((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :bottom-left-contact))
       (equal floor-contact-state :released)
       (find :bottom-left contact-surface)
       (null (find :bottom contact-surface)))
  :bottom-left-push)
 ((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :front-left-contact))
       (equal floor-contact-state :bottom-contact)
       (find :front-left contact-surface)
       (find :bottom contact-surface))
  :front-left-push)
 ((and (equal larm-contact-state :released)
       (null (equal rarm-contact-state :back-left-contact))
       (equal floor-contact-state :bottom-contact)
       (find :back-left contact-surface)
       (find :bottom contact-surface))
  :back-left-push)
 ((or (and (equal larm-contact-state :left-contact)
           (equal rarm-contact-state :released)
           (equal floor-contact-state :bottom-contact)
           (find :left contact-surface)
           (find :bottom contact-surface)
           (null (find :right bottom-contact-position)))
      (and (equal larm-contact-state :left-contact)
           (equal rarm-contact-state :released)
           (equal floor-contact-state :bottom-right-side-contact)
           (find :left contact-surface)
           (find :bottom contact-surface)
           (find :left bottom-contact-position)))
  :left-move-push)
 ((and (equal larm-contact-state :released)
       (or (equal rarm-contact-state :right-contact)
           (equal rarm-contact-state :released))
       (equal floor-contact-state :bottom-right-side-contact)
       ; (find :right contact-surface)
       (find :bottom contact-surface)
       (find :left bottom-contact-position))
  :bottom-left-side-push)
 ((and (equal larm-contact-state :released)
       (equal rarm-contact-state :bottom-right-side-contact)
       (equal floor-contact-state :bottom-left-side-contact)
       (find :top contact-surface)
       (find :bottom contact-surface))
  :top-right-side-push)
 ((or (and (equal larm-contact-state :left-contact)
           (or (equal floor-contact-state :bottom-contact)
               (equal floor-contact-state :bottom-left-side-contact)
               (equal floor-contact-state :bottom-right-side-contact))
           (null (or (find :left contact-surface)
                     (find :top-left contact-surface)
                     (find :bottom-left contact-surface)
                     (find :front-left contact-surface)
                     (find :back-left contact-surface)))
           (find :bottom contact-surface))
      (and (equal larm-contact-state :front-contact)
           (or (equal floor-contact-state :bottom-contact)
               (equal floor-contact-state :bottom-left-side-contact)
               (equal floor-contact-state :bottom-right-side-contact))
           (null (or (find :front contact-surface)
                     (find :front-left contact-surface)
                     (find :front-right contact-surface)
                     (find :top-front contact-surface)
                     (find :bottom-front contact-surface)))
           (find :bottom contact-surface))
      (and (equal larm-contact-state :back-contact)
           (or (equal floor-contact-state :bottom-contact)
               (equal floor-contact-state :bottom-left-side-contact)
               (equal floor-contact-state :bottom-right-side-contact))
           (null (or (find :back contact-surface)
                     (find :back-left contact-surface)
                     (find :back-right contact-surface)
                     (find :top-back contact-surface)
                     (find :bottom-back contact-surface)))
           (find :bottom contact-surface))
      (and (equal larm-contact-state :top-contact)
           (or (equal floor-contact-state :bottom-contact)
               (equal floor-contact-state :bottom-left-side-contact)
               (equal floor-contact-state :bottom-right-side-contact))
           (null (or (find :top contact-surface)
                     (find :top-left contact-surface)
                     (find :top-right contact-surface)
                     (find :top-front contact-surface)
                     (find :top-back contact-surface)))
           (find :bottom contact-surface)
           (null (find :left contact-surface)))
      (and (equal larm-contact-state :top-left-contact)
           (equal floor-contact-state :released)
           (null (or (find :top-left contact-surface)
                     (find :top contact-surface)
                     (find :left contact-surface))))
      (and (equal larm-contact-state :bottom-left-contact)
           (equal floor-contact-state :released)
           (null (or (find :bottom-left contact-surface)
                     (find :bottom contact-surface)
                     (find :left contact-surface))))
      (and (equal larm-contact-state :front-left-contact)
           (equal floor-contact-state :bottom-contact)
           (null (or (find :front-left contact-surface)
                     (find :left contact-surface)
                     (find :front contact-surface)))
           (find :bottom contact-surface))
      (and (equal larm-contact-state :back-left-contact)
           (equal floor-contact-state :bottom-contact)
           (null (or (find :back-left contact-surface)
                     (find :left contact-surface)
                     (find :back contact-surface)))
           (find :bottom contact-surface))
      (and (equal larm-contact-state :bottom-left-side-contact)
           (equal floor-contact-state :bottom-right-side-contact)
           (find :bottom contact-surface)
           (null (find :left contact-surface))
           (null (find :left bottom-contact-position)))
      (and (equal larm-contact-state :top-right-side-contact)
           (equal floor-contact-state :bottom-left-side-contact)
           (find :bottom contact-surface)
           (null (find :left contact-surface))
           (null (find :top contact-surface)))
      (and (equal larm-contact-state :left-top-side-contact)
           (equal floor-contact-state :bottom-contact)
           (null (find :left contact-surface))
           (find :bottom contact-surface)))
  :push-release))