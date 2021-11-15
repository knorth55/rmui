(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


;; parameters

(defun box-scene-init (&optional (n-box 1))
  (let ((boxes nil))
    (dotimes (i n-box)
      (let* ((box-id (format nil "box~A" i))
            (box (make-cube 270 420 220 :name box-id))  ;; ctb
            ;; (box (make-cube 350 350 300 :name box-id)) ;; cardboard box
            ;; (box (make-cube 270 370 500 :name box-id))  ;; trash box
            ;; (box (make-cube 150 280 320 :name box-id))  ;; shredder
            (box-pos-z (+ (+ *table-z* *table-height*) (/ (z-of-cube box) 2.0)))
            (box-pos-y (* i (+ (y-of-cube box) 100)))
            (box-pos (float-vector *box-pos-x* box-pos-y box-pos-z)))
        (send box :reset-coords)
        (send box :worldcoords)
        (send box :newcoords (make-coords :pos box-pos))
        (send box :worldcoords)
        (eval `(defvar ,(read-from-string (format nil "*box~A*" i)) box))
        (setq boxes (append boxes (list box)))
        ))
    (defvar *boxes* boxes)))


(defun table-scene-init (table-pos-y)
  (defvar *table-id* "room73b2-askul-1200x700-desk-0")
  (defvar *table* (room73b2-askul-1200x700-desk-0))
  (defvar *table-pos* (float-vector (+ *box-pos-x* 100) table-pos-y *table-z*))
  (send *table* :reset-coords)
  (send *table* :worldcoords)
  (send *table* :newcoords (make-coords :pos *table-pos*))
  (send *table* :worldcoords))


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


(defun scene-init (&key (table-pos-y 0) (n-box 1))
  (table-scene-init table-pos-y)
  (box-scene-init n-box)
  (robot-scene-init :table-objects (append *boxes* (list *table*))
                    :table-pos *table-pos*)
  ;: viewer-init
  (make-irtviewer :title "Robot Viewer")
  (objects (append *boxes* (list *robot* *table*)) *irtviewer*))


(provide :common-scenes "common-scenes.l")
