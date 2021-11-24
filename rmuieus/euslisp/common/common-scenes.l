(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")


(defun get-initial-scene-states (&optional (n-box))
  (let ((scene-states nil)
        (cube-contact-state nil))
    (dotimes (i n-box)
      (setq cube-contact-state
            (make-list n-box :initial-element :released))
      (setf (elt cube-contact-state i) :never)
      (setq scene-states
            (append scene-states
                    (list
                      (list
                        (list :larm-contact-state :released)
                        (list :rarm-contact-state :released)
                        (list :floor-contact-state :bottom-contact)
                        (list :object-location-state :center)
                        (list :object-rotation-state (list :x0 :y0 :z0))
                        (list :cube-contact-state cube-contact-state)
                        (list :left-wall-contact-state :released)
                        (list :front-wall-contact-state :released)
                        )))))
    scene-states))


(defun box-scene-init (&optional (n-box 1) (box-types :ctb))
  (let ((boxes nil)
        (colors (list :red :green :blue :yellow :pink :purple))
        (box-types
          (if (listp box-types)
            box-types
            (make-list n-box :initial-element box-types)))
        (box-shapes
          (list
            (cons :ctb (list 220 210 200))  ;; ctb
            (cons :cardboardbox (list 350 350 300))  ;; cardboard box
            (cons :trashbox (list 270 370 500))  ;; trash box
            (cons :shredder (list 150 280 320))  ;; shredder
            (cons :toybox (list 200 200 200))  ;; toy box for debugging
            )))
    (dotimes (i n-box)
      (let* ((box-id (format nil "box~A" i))
             (box-type (elt box-types i))
             (box-shape (cdr (assoc box-type box-shapes)))
             (box (apply #'make-cube (append box-shape (list :name box-id))))
             (box-pos-z (+ (+ *table-z* *table-height*) (/ (z-of-cube box) 2.0)))
             (box-pos-y (* (if (oddp i) 1 -1)
                           (floor (/ (+ i 1) 2))
                           (+ (y-of-cube box) 100)))
             (box-pos (float-vector *box-pos-x* box-pos-y box-pos-z)))
        (send box :set-color (elt colors (mod i 6)))
        (send box :reset-coords)
        (send box :worldcoords)
        (send box :newcoords (make-coords :pos box-pos))
        (send box :worldcoords)
        (send box :put :initial-pos (copy-seq box-pos))
        (eval `(defvar ,(read-from-string (format nil "*box~A*" i)) box))
        (setq boxes (append boxes (list box)))
        ))
    (defvar *boxes* boxes)))


(defun table-scene-init (table-pos-y)
  (defvar *table-id* "room73b2-askul-1200x700-desk-0")
  (defvar *table* (room73b2-askul-1200x700-desk-0))
  (defvar *table-height* 700)
  (defvar *table-pos*
    (float-vector (+ *box-pos-x* 100) table-pos-y *table-z*))
  (send *table* :reset-coords)
  (send *table* :worldcoords)
  (send *table* :newcoords (make-coords :pos *table-pos*))
  (send *table* :worldcoords))


(defun wall-scene-init ()
  (let* ((wall-height 200)
         (wall-width 10)
         (left-wall-length 300)
         (front-wall-length 1000)
         (left-wall-pos
           (float-vector
             *box-pos-x*
             (+ (/ front-wall-length 2.0) (/ wall-width 2.0))
             (+ *table-z* *table-height* (/ wall-height 2.0))
             ))
         (front-wall-pos
           (float-vector
             (+ *box-pos-x*
                (/ left-wall-length 2.0))
             0
             (+ *table-z* *table-height* (/ wall-height 2.0)))))
    (defvar *left-wall*
      (make-cube left-wall-length wall-width wall-height))
    (defvar *front-wall*
      (make-cube wall-width front-wall-length wall-height))
    (send *left-wall* :reset-coords)
    (send *left-wall* :worldcoords)
    (send *left-wall* :newcoords (make-coords :pos left-wall-pos))
    (send *left-wall* :worldcoords)
    (send *front-wall* :reset-coords)
    (send *front-wall* :worldcoords)
    (send *front-wall* :newcoords (make-coords :pos front-wall-pos))
    (send *front-wall* :worldcoords)
    (defvar *wall-front-limit*
      (- (elt front-wall-pos 0) (/ wall-width 2.0)))
    (defvar *wall-left-limit*
      (- (elt left-wall-pos 1) (/ wall-width 2.0)))
    ))


(defun pilot-device-scene-init (&optional (n-device 1))
  (let ((pilot-devices nil)
        (colors (list :red :green :blue :yellow :pink :purple)))
    (dotimes (i n-device)
      (let* ((pilot-device-id (format nil "pilot-device~A" i))
             (pilot-device (make-cube 100 100 100 :name pilot-device-id))
             (pilot-device-pos-z (+ (+ *pilot-table-z* *pilot-table-height*)
                                    (/ (z-of-cube pilot-device) 2.0)))
             (pilot-device-pos-y (* (if (oddp i) 1 -1)
                                    (floor (/ (+ i 1) 2))
                                    (+ (y-of-cube pilot-device) 100)))
             (pilot-device-pos
               (float-vector *pilot-device-pos-x* pilot-device-pos-y pilot-device-pos-z)))
        (send pilot-device :set-color (elt colors (mod i 6)))
        (send pilot-device :reset-coords)
        (send pilot-device :worldcoords)
        (send pilot-device :newcoords (make-coords :pos pilot-device-pos))
        (send pilot-device :worldcoords)
        (send pilot-device :put :initial-pos (copy-seq pilot-device-pos))
        (eval `(defvar ,(read-from-string (format nil "*pilot-device~A*" i)) pilot-device))
        (setq pilot-devices (append pilot-devices (list pilot-device)))
        ))
    (defvar *pilot-devices* pilot-devices)))


(defun pilot-table-scene-init (table-pos-y)
  (defvar *pilot-table-id* "room73b2-askul-1200x700-desk-0")
  (defvar *pilot-table* (room73b2-askul-1200x700-desk-0))
  (defvar *pilot-table-height* 700)
  (defvar *pilot-table-pos*
    (float-vector (+ *pilot-device-pos-x* 100) (* -1 table-pos-y) *pilot-table-z*))
  (send *pilot-table* :reset-coords)
  (send *pilot-table* :worldcoords)
  (send *pilot-table* :newcoords (make-coords :pos *pilot-table-pos*))
  (send *pilot-table* :worldcoords))


(defun pilot-wall-scene-init ()
  (let* ((wall-height 200)
         (wall-width 10)
         (left-wall-length 300)
         (front-wall-length 800)
         (left-wall-pos
           (float-vector
             *pilot-device-pos-x*
             (+ (/ front-wall-length 2.0) (/ wall-width 2.0))
             (+ *pilot-table-z* *pilot-table-height* (/ wall-height 2.0))
             ))
         (front-wall-pos
           (float-vector
             (+ *pilot-device-pos-x*
                (/ left-wall-length 2.0))
             0
             (+ *pilot-table-z* *pilot-table-height* (/ wall-height 2.0))))
         )
    (defvar *pilot-left-wall*
      (make-cube left-wall-length wall-width wall-height))
    (defvar *pilot-front-wall*
      (make-cube wall-width front-wall-length wall-height))
    (send *pilot-left-wall* :reset-coords)
    (send *pilot-left-wall* :worldcoords)
    (send *pilot-left-wall* :newcoords (make-coords :pos left-wall-pos))
    (send *pilot-left-wall* :worldcoords)
    (send *pilot-front-wall* :reset-coords)
    (send *pilot-front-wall* :worldcoords)
    (send *pilot-front-wall* :newcoords (make-coords :pos front-wall-pos))
    (send *pilot-front-wall* :worldcoords)
    (defvar *pilot-wall-front-limit*
      (- (elt front-wall-pos 0) (/ wall-width 2.0)))
    (defvar *pilot-wall-left-limit*
      (- (elt left-wall-pos 1) (/ wall-width 2.0)))
    ))


(defun pilot-robot-scene-init-base (&key (table-pos)
                                         (larm-ee-frame-id)
                                         (rarm-ee-frame-id)
                                         (larm-contact-coords)
                                         (rarm-contact-coords))
  (if (null larm-contact-coords)
    (setq larm-contact-coords
          (send (send *pilot* :larm-end-coords) :copy-worldcoords)))
  (if (null rarm-contact-coords)
    (setq rarm-contact-coords
          (send (send *pilot* :rarm-end-coords) :copy-worldcoords)))
  (send larm-contact-coords :translate *pilot-contact-offset*)
  (send rarm-contact-coords :translate *pilot-contact-offset*)

  (send *pilot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *pilot* :link larm-ee-frame-id)
                        :coords larm-contact-coords))
  (send *pilot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *pilot* :link rarm-ee-frame-id)
                        :coords rarm-contact-coords)))


(defun robot-scene-init-base (&key (table-pos)
                                   (larm-ee-frame-id)
                                   (rarm-ee-frame-id)
                                   (larm-contact-coords)
                                   (rarm-contact-coords)
                                   (base-frame-id))
  ;; (send (send *robot* :larm :end-coords) :translate *contact-offset*)
  ;; (send (send *robot* :rarm :end-coords) :translate *contact-offset*)
  ;; add :larm-contact-coords and :rarm-contact-coords
  (if (null larm-contact-coords)
    (setq larm-contact-coords
          (send (send *robot* :larm-end-coords) :copy-worldcoords)))
  (if (null rarm-contact-coords)
    (setq rarm-contact-coords
          (send (send *robot* :rarm-end-coords) :copy-worldcoords)))
  (send larm-contact-coords :translate *contact-offset*)
  (send rarm-contact-coords :translate *contact-offset*)

  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link larm-ee-frame-id)
                        :coords larm-contact-coords))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link rarm-ee-frame-id)
                        :coords rarm-contact-coords))
  (send (send *ri* :robot) :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send (send *ri* :robot) :link larm-ee-frame-id)
                        :coords larm-contact-coords))
  (send (send *ri* :robot) :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send (send *ri* :robot) :link rarm-ee-frame-id)
                        :coords rarm-contact-coords))

  (send *robot* :put :base-frame-id base-frame-id)
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
            :relative-pose (make-coords :pos table-pos)
            :object-id "table")))
  t)


(defun scene-init (&key (table-pos-y 0) (n-box 1) (box-types :ctb)
                        (wall-p t))
  (table-scene-init table-pos-y)
  (box-scene-init n-box box-types)
  (if wall-p (wall-scene-init))
  (robot-scene-init
    :table-pos *table-pos*
    :table-objects
    (append *boxes*
            (list *table* *left-wall* *front-wall*)))
  ;: viewer-init
  (make-irtviewer :title "Robot Viewer")
  (objects (append *boxes*
                   (list *robot* *table*
                         *left-wall* *front-wall*))
           *irtviewer*))


(defun pilot-scene-init (&key (table-pos-y 0) (n-device 1) (wall-p t))
  (pilot-table-scene-init table-pos-y)
  (pilot-device-scene-init n-device)
  (if wall-p (pilot-wall-scene-init))
  (pilot-robot-scene-init :table-pos *table-pos*)
  ;: viewer-init
  (setq *pilotviewer* (instance x::irtviewer :create :title "Pilot Viewer"))
  (objects (append *pilot-devices*
                   (list *pilot* *pilot-table*
                         *pilot-left-wall* *pilot-front-wall*))
           *pilotviewer*))


(provide :common-scenes "common-scenes.l")
