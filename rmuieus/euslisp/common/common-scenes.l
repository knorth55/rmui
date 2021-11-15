(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


;; parameters
(defvar *box-id* "box")
;; (defvar *box* (make-cube 350 350 300 :name *box-id*)) ;; cardboard box
(defvar *box* (make-cube 270 420 220 :name *box-id*))  ;; ctb
;; (defvar *box* (make-cube 270 370 500 :name *box-id*))  ;; trash box
;; (defvar *box* (make-cube 150 280 320 :name *box-id*))  ;; shredder
(defvar *table-id* "room73b2-askul-1200x700-desk-0")
(defvar *table* (room73b2-askul-1200x700-desk-0))


(defun set-scene-parameters (box-pos-y)
  (defvar *box-pos-z* (+ (+ *table-z* *table-height*) (/ (z-of-cube *box*) 2.0)))
  (defvar *box-pos* (float-vector *box-pos-x* 0 *box-pos-z*))
  (defvar *table-pos* (float-vector (+ *box-pos-x* 100) (* -1 box-pos-y) *table-z*)))


(defun table-scene-init (table-pos)
  (send *table* :reset-coords)
  (send *table* :worldcoords)
  (send *table* :newcoords (make-coords :pos table-pos))
  (send *table* :worldcoords))


(defun box-scene-init (box-pos)
  (send *box* :reset-coords)
  (send *box* :worldcoords)
  (send *box* :newcoords (make-coords :pos box-pos))
  (send *box* :worldcoords))


(defun robot-scene-init-base (&key (table-objects) (table-pos)
                                   (larm-ee-frame-id)
                                   (rarm-ee-frame-id)
                                   (base-frame-id))
  ;; (send (send *robot* :larm :end-coords) :translate *contact-offset*)
  ;; (send (send *robot* :rarm :end-coords) :translate *contact-offset*)
  ;; add :larm-contact-coords and :rarm-contact-coords
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link larm-ee-frame-id)
                        :coords (send (send (send *robot* :larm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link rarm-ee-frame-id)
                        :coords (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                                      :translate *contact-offset*)))
  (send (send *ri* :robot) :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send (send *ri* :robot) :link larm-ee-frame-id)
                        :coords
                        (send (send (send (send *ri* :robot) :larm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (send (send *ri* :robot) :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send (send *ri* :robot) :link rarm-ee-frame-id)
                        :coords
                        (send (send (send (send *ri* :robot) :rarm-end-coords) :copy-worldcoords)
                              :translate *contact-offset*)))
  (if (send *ri* :simulation-modep)
    (progn
      (send (get (geo:find-viewer (send *robot* :name)) :pickviewer)
            :draw-floor nil)
      (send (get (geo:find-viewer (send *robot* :name)) :pickviewer)
            :change-background (float-vector 0.1 0.1 0.1)))
    (progn
      (setq *co* (instance collision-object-publisher :init))
      (send *co* :wipe-all)
      (send *co* :add-object *table* :frame-id base-frame-id
            :relative-pose (make-coords :pos table-pos)))))


(defun scene-init (&key (box-pos-y 0))
  (set-scene-parameters box-pos-y)
  (table-scene-init *table-pos*)
  (box-scene-init *box-pos*)
  (robot-scene-init :table-objects (list *box* *table*)
                    :table-pos *table-pos*)
  ;: viewer-init
  (make-irtviewer :title "Robot Viewer")
  (objects (list *robot* *box* *table*) *irtviewer*))


(provide :common-scenes "common-scenes.l")
