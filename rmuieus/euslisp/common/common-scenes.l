(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")
(require :collision-object-publisher "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")


(defvar *box-shapes*
  (list (cons :ctb (list 270 420 220))  ;; ctb
        (cons :jetson (list 220 200 200))  ;; jetson box
        (cons :cardboardbox (list 350 350 300))  ;; cardboard box
        (cons :trashbox (list 270 370 500))  ;; trash box
        (cons :shredder (list 150 280 320))  ;; shredder
        (cons :arccardboardbox1 (list 270 350 120))  ;; arc cardboard box1
        (cons :arccardboardbox2 (list 350 410 160))  ;; arc cardboard box2
        (cons :toybox (list 200 200 200))  ;; toy box for debugging
        (cons :toyobject (list 100 100 100)) ;; toy object for debugging
        ))


(defun set-reset-state (&optional (boxes *boxes*))
  (defvar *reset-state* (get-initial-scene-states boxes))
  (ros::ros-info "reset-state: ~A" *reset-state*))


(defun get-initial-scene-states (&optional (boxes *boxes*))
  (let ((n-box (length boxes))
        (scene-states nil)
        (cube-contact-state nil))
    (dotimes (i n-box)
      (let* ((box (elt boxes i))
             (box-type (send box :get :box-type))
             (box-rotation-angle
               (rotation-angle (send box :worldrot)))
             ;; TODO: support only z axis rotation
             ;; support only z0 or z45
             (box-z-rotation-state
               (if (and box-rotation-angle
                        (null (= (elt (cadr box-rotation-angle) 2) 0)))
                 (let ((box-z-rotation-angle
                         (mod (if (> (elt (cadr box-rotation-angle) 2) 0)
                                (car box-rotation-angle)
                                (- (* pi 2) (car box-rotation-angle)))
                              (/ pi 2))))
                   (if (and (> box-z-rotation-angle (/ pi 8))
                            (< box-z-rotation-angle (* (/ pi 8) 3)))
                     :z45 :z0))
                 :z0)))
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
                          (list :object-rotation-state
                                (list :x0 :y0 box-z-rotation-state))
                          (list :cube-contact-state cube-contact-state)
                          (list :left-wall-contact-state :released)
                          (list :front-wall-contact-state :released)))))))
    scene-states))


(defun bbox->cube (bb &key (base-frame-id))
  (let* ((dims (ros::tf-point->pos (send bb :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
         (bb-frame-id (send bb :header :frame_id))
         (bb-coords (ros::tf-pose->coords (send bb :pose)))
         (bx-coords
           (if (and base-frame-id
                    (not (equal base-frame-id bb-frame-id)))
             (send (send *tfl* :lookup-transform
                         base-frame-id bb-frame-id (ros::time 0))
                   :transform bb-coords)
             bb-coords)))
    ;; CPI decomposer's z is downside
    (send bx-coords :rotate pi :x)
    (send bx :newcoords bx-coords)
    bx))


(defun wait-for-publisher
  (topic-name topic-type &optional (timeout 10))
  (let ((topic-count 0))
    (ros::ros-info "Waiting ~A" topic-name)
    (ros::subscribe topic-name topic-type
                    '(lambda-closure nil 0 0 (msg) t))
    (unix::usleep (* 100 1000))
    (ros::rate 10)
    (while (= (ros::get-num-publishers topic-name) 0)
      (when (and (> topic-count 0) (= (mod topic-count 50) 0))
        (ros::ros-warn "Waiting ~A for ~A seconds"
                       topic-name (/ topic-count 10)))
      (when (and timeout
                 (>= topic-count (* timeout 10)))
        (ros::ros-warn "[~A] topic is not found" topic-name)
        (return-from wait-for-publisher nil))
      (incf topic-count)
      (unix::usleep (* 100 1000))
      (ros::spin-once))
    (ros::unsubscribe topic-name)
    t))


(defun recognize-tabletop-boxes
  (&key (stamp (ros::time-now)) (timeout 30) (base-frame-id))
  (let* ((decomposer-name "/tabletop_cpi_decomposer")
         (bbox-topic-name (format nil "~A/boxes" decomposer-name))
         (bbox-topic-type jsk_recognition_msgs::BoundingBoxArray)
         (bbox-is-publish
           (wait-for-publisher bbox-topic-name bbox-topic-type))
         (bbox-msg
           (if bbox-is-publish
             (one-shot-subscribe
               bbox-topic-name bbox-topic-type
               :timeout (if timeout (round (* timeout 1000)))
               :after-stamp stamp))))
    (if bbox-msg
      (mapcar #'(lambda (bb) (bbox->cube bb :base-frame-id base-frame-id))
              (send bbox-msg :boxes)))))


(defun create-box (&key (box-type) (box-shapes) (box-pos) (box-rpy) (box-id))
  (if (null box-rpy) (setq box-rpy (float-vector 0 0 0)))
  (if (null box-shapes) (setq box-shapes (copy-tree *box-shapes*)))
  (let* ((box-shape (cdr (assoc box-type box-shapes)))
         (box (apply #'make-cube (append box-shape (list :name box-id)))))
    (setf (elt box-pos 2) (+ (elt box-pos 2) (/ (z-of-cube box) 2.0)))
    (send box :put :box-type box-type)
    (send box :reset-coords)
    (send box :worldcoords)
    (send box :newcoords (make-coords :pos box-pos :rpy box-rpy))
    (send box :worldcoords)
    (send box :put :initial-pos (copy-seq box-pos))
    (send box :put :initial-rpy (copy-seq box-rpy))
    box))


(defun get-box (&key (box-type) (box-pos) (box-rpy) (box-id) (tabletop-boxes))
  (let* ((box-shapes (copy-tree *box-shapes*))
         (box-type-names
           (mapcar #'(lambda (box-shape) (car box-shape)) box-shapes)))
    (cond
      ((find box-type box-type-names :test #'equal)
       (return-from get-box
                    (create-box
                      :box-type box-type
                      :box-shapes box-shapes
                      :box-pos box-pos
                      :box-rpy box-rpy
                      :box-id box-id)))
      ;; TODO: support more object for recognize
      ((find box-type (list :object :largeobject :stuffedanimal) :test #'equal)
       (if (null tabletop-boxes)
         (return-from get-box
                      (create-box
                        :box-type
                        (cond
                          ((find box-type (list :largeobject :stuffedanimal) :test #'equal)
                           :toybox)
                          ((equal box-type :object)
                           :toyobject)
                          (t nil))
                        :box-shapes box-shapes
                        :box-pos box-pos
                        :box-rpy box-rpy
                        :box-id box-id)))
       (let* ((tabletop-box
                (copy-object (car (sort-close-boxes tabletop-boxes :pos box-pos)))))
         (if tabletop-box
           (let ((tabletop-box-pos
                   (copy-seq (send tabletop-box :worldpos)))
                 (tabletop-box-rpy))
             ;; TODO: support only z axis rotation
             (if (find box-type (list :largeobject :stuffedanimal) :test #'equal)
               (let* ((tabletop-box-rotation-angle
                        (rotation-angle (send tabletop-box :worldrot)))
                      (tabletop-box-z-angle
                        (* (if (> (elt (cadr tabletop-box-rotation-angle) 2) 0) 1 -1)
                           (car tabletop-box-rotation-angle)))
                      (tabletop-box-coords
                        (send (send tabletop-box :worldcoords) :copy-worldcoords))
                      (tabletop-box-x (x-of-cube tabletop-box))
                      (tabletop-box-y (y-of-cube tabletop-box))
                      (tabletop-box-z
                        (- (+ (elt tabletop-box-pos 2) (/ (z-of-cube tabletop-box) 2))
                           (+ *table-height* *table-z*)))
                      (tabletop-box-xy-offset
                        (if (equal box-type :stuffedanimal) -100 0))
                      (tabletop-box-z-offset
                        (if (equal box-type :stuffedanimal) -50 0))
                      )
                 (cond
                   ((> tabletop-box-z-angle (+ (if box-rpy (elt box-rpy 0) 0)
                                               (/ pi 4)))
                    (setq tabletop-box
                          (make-cube
                            (max 50 (+ tabletop-box-y tabletop-box-xy-offset))
                            (max 50 (+ tabletop-box-x tabletop-box-xy-offset))
                            (max 50 (+ tabletop-box-z tabletop-box-z-offset))))
                    (setf (elt tabletop-box-pos 2)
                          (+ (/ (z-of-cube tabletop-box) 2) *table-height* *table-z*))
                    (if box-rpy
                      (progn
                        (send tabletop-box :newcoords
                              (make-coords :pos tabletop-box-pos :rpy box-rpy))
                        (send tabletop-box :worldcoords))
                      (progn
                        (send tabletop-box-coords :rotate (/ -pi 2) :z :world)
                        (send tabletop-box-coords :locate tabletop-box-pos :world)
                        (send tabletop-box :newcoords tabletop-box-coords)
                        (send tabletop-box :worldcoords))))
                   ((< tabletop-box-z-angle (+ (if box-rpy (elt box-rpy 0) 0)
                                               (/ -pi 4)))
                    (setq tabletop-box
                          (make-cube
                            (max 50 (+ tabletop-box-y tabletop-box-xy-offset))
                            (max 50 (+ tabletop-box-x tabletop-box-xy-offset))
                            (max 50 (+ tabletop-box-z tabletop-box-z-offset))))
                    (setf (elt tabletop-box-pos 2)
                          (+ (/ (z-of-cube tabletop-box) 2) *table-height* *table-z*))
                    (if box-rpy
                      (progn
                        (send tabletop-box :newcoords
                              (make-coords :pos tabletop-box-pos :rpy box-rpy))
                        (send tabletop-box :worldcoords))
                      (progn
                        (send tabletop-box-coords :rotate (/ pi 2) :z :world)
                        (send tabletop-box-coords :locate tabletop-box-pos :world)
                        (send tabletop-box :newcoords tabletop-box-coords)
                        (send tabletop-box :worldcoords))))
                   (t
                     (setq tabletop-box
                           (make-cube
                             (max 50 (+ tabletop-box-x tabletop-box-xy-offset))
                             (max 50 (+ tabletop-box-y tabletop-box-xy-offset))
                             (max 50 (+ tabletop-box-z tabletop-box-z-offset))))
                     (setf (elt tabletop-box-pos 2)
                           (+ (/ (z-of-cube tabletop-box) 2) *table-height* *table-z*))
                     (if box-rpy
                       (progn
                         (send tabletop-box :newcoords
                               (make-coords :pos tabletop-box-pos :rpy box-rpy))
                         (send tabletop-box :worldcoords))
                       (progn
                         (send tabletop-box-coords :locate tabletop-box-pos :world)
                         (send tabletop-box :newcoords tabletop-box-coords)
                         (send tabletop-box :worldcoords))))
                   )))
             (setq tabletop-box-rpy
                   (apply #'float-vector
                          (car (rpy-angle (send tabletop-box :worldrot)))))
             (send tabletop-box :name box-id)
             (send tabletop-box :put :box-type box-type)
             (send tabletop-box :put :initial-pos tabletop-box-pos)
             (send tabletop-box :put :initial-rpy tabletop-box-rpy)
             (return-from get-box tabletop-box))
           (progn
             (ros::ros-error "No bbox is recognized close to object position")
             (return-from get-box nil)))))
      (t
        (ros::ros-error "Invalid box type: ~A" box-type)
        (return-from get-box nil)))))


(defun sort-close-boxes (boxes &key (pos #f(0 0 0)))
  (let ((sorted-boxes (copy-object boxes)))
    (sort sorted-boxes #'<
          #'(lambda (box)
              (let ((box-xy-pos (copy-seq (send box :worldpos)))
                    (xy-pos (copy-seq pos)))
                ;; compare only x,y axis
                (setf (elt box-xy-pos 2) 0)
                (setf (elt xy-pos 2) 0)
                (norm (v- box-xy-pos xy-pos)))))
    sorted-boxes))


(defun box-scene-init (scene-alist &key (base-frame-id))
  (let* ((boxes nil)
         (colors (list :red :green :blue :yellow :pink :purple))
         ;; TODO: support more object for recognize
         (recognize-tabletop
           (some #'(lambda (x) (find x (list :object :largeobject :stuffedanimal) :test #'equal))
                 (mapcar #'(lambda (scene-a) (cadr (assoc :box-type scene-a))) scene-alist)))
         (tabletop-boxes
           (if recognize-tabletop
             (recognize-tabletop-boxes :base-frame-id base-frame-id))))
    (dotimes (i (length scene-alist))
      (let* ((scene-a (elt scene-alist i))
             (box-id (format nil "box~A" i))
             (box-type (cadr (assoc :box-type scene-a)))
             (box-pos-a (cadr (assoc :box-pos scene-a)))
             (box-pos-x (+ *box-pos-x* (elt box-pos-a 0)))
             (box-pos-y (elt box-pos-a 1))
             (box-pos-z (+ (+ *table-z* *table-height*)
                           (elt box-pos-a 2)))
             (box-pos (float-vector box-pos-x box-pos-y box-pos-z))
             (box-rpy-a (cadr (assoc :box-rpy scene-a)))
             (box-rpy (if box-rpy-a
                        (float-vector (eval (elt box-rpy-a 0))
                                      (eval (elt box-rpy-a 1))
                                      (eval (elt box-rpy-a 2)))))
             (box (get-box
                    :box-type box-type
                    :box-pos box-pos
                    :box-rpy box-rpy
                    :box-id box-id
                    :tabletop-boxes tabletop-boxes)))
        (if (null box-rpy)
          (setf (cadr (assoc :box-rpy (elt scene-alist i)))
                (car (rpy-angle (send box :worldrot)))))
        (send box :set-color (elt colors (mod i 6)))
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


(defun pilot-device-scene-init (scene-alist)
  (let ((devices nil)
        (colors (list :red :green :blue :yellow :pink :purple))
        (device-shapes
          (list (cons :mtc1 (list 120 120 120))  ;; miniature tangible cube rev.1
                (cons :mtc2 (list 100 100 100))  ;; miniature tangible cube rev.2
                )))
    (dotimes (i (length scene-alist))
      (let* ((scene-a (elt scene-alist i))
             (device-id (format nil "pilot-device~A" i))
             (device-type (cadr (assoc :device-type scene-a)))
             (device-shape (cdr (assoc device-type device-shapes)))
             (device (apply #'make-cube (append device-shape (list :name device-id))))
             (device-pos-a (cadr (assoc :device-pos scene-a)))
             (device-pos-x (+ *pilot-device-pos-x* (elt device-pos-a 0)))
             (device-pos-y (elt device-pos-a 1))
             (device-pos-z (+ (+ *pilot-table-z* *pilot-table-height*)
                              (/ (z-of-cube device) 2.0)
                              (elt device-pos-a 2)))
             (device-pos
               (float-vector device-pos-x device-pos-y device-pos-z))
             (box-rpy-a (cadr (assoc :box-rpy scene-a)))
             (device-rpy-a (cadr (assoc :device-rpy scene-a)))
             (device-rpy (if device-rpy-a
                           (float-vector (eval (elt device-rpy-a 0))
                                         (eval (elt device-rpy-a 1))
                                         (eval (elt device-rpy-a 2)))
                           (float-vector (eval (elt box-rpy-a 0))
                                         (eval (elt box-rpy-a 1))
                                         (eval (elt box-rpy-a 2))))))
        (send device :put :device-type device-type)
        (send device :set-color (elt colors (mod i 6)))
        (send device :reset-coords)
        (send device :worldcoords)
        (send device :newcoords
              (make-coords :pos device-pos :rpy device-rpy))
        (send device :worldcoords)
        (send device :put :initial-pos (copy-seq device-pos))
        (send device :put :initial-rpy (copy-seq device-rpy))
        (eval `(defvar ,(read-from-string (format nil "*pilot-device~A*" i)) device))
        (setq devices (append devices (list device)))
        ))
    (defvar *pilot-devices* devices)))


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
                                         (rarm-contact-coords)
                                         (larm-grasp-coords)
                                         (rarm-grasp-coords))
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
                        :coords rarm-contact-coords))
  (send *pilot* :put :larm-grasp-coords
        (make-cascoords :name :larm-grasp-coords
                        :parent (send *pilot* :link larm-ee-frame-id)
                        :coords larm-grasp-coords))
  (send *pilot* :put :rarm-grasp-coords
        (make-cascoords :name :rarm-grasp-coords
                        :parent (send *pilot* :link rarm-ee-frame-id)
                        :coords rarm-grasp-coords))
  )


(defun robot-scene-init-base (&key (table-pos)
                                   (larm-ee-frame-id)
                                   (rarm-ee-frame-id)
                                   (larm-contact-coords)
                                   (rarm-contact-coords)
                                   (larm-grasp-coords)
                                   (rarm-grasp-coords)
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
  (if (null larm-grasp-coords)
    (setq larm-grasp-coords
          (send (send *robot* :larm-end-coords) :copy-worldcoords)))
  (if (null rarm-grasp-coords)
    (setq rarm-grasp-coords
          (send (send *robot* :rarm-end-coords) :copy-worldcoords)))

  (send larm-contact-coords :translate *contact-offset*)
  (send rarm-contact-coords :translate *contact-offset*)

  ;; contact coords
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

  ;; grasp coords
  (send *robot* :put :larm-grasp-coords
        (make-cascoords :name :larm-grasp-coords
                        :parent (send *robot* :link larm-ee-frame-id)
                        :coords larm-grasp-coords))
  (send *robot* :put :rarm-grasp-coords
        (make-cascoords :name :rarm-grasp-coords
                        :parent (send *robot* :link rarm-ee-frame-id)
                        :coords rarm-grasp-coords))
  (send (send *ri* :robot) :put :larm-grasp-coords
        (make-cascoords :name :larm-grasp-coords
                        :parent (send (send *ri* :robot) :link larm-ee-frame-id)
                        :coords larm-grasp-coords))
  (send (send *ri* :robot) :put :rarm-grasp-coords
        (make-cascoords :name :rarm-grasp-coords
                        :parent (send (send *ri* :robot) :link rarm-ee-frame-id)
                        :coords rarm-grasp-coords))

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


(defun scene-init (&key (table-pos-y 0) (scene-alist nil) (wall-p nil))
  (if (null scene-alist)
    (setq scene-alist (load-scene-alist "one_ctb")))
  (let ((table-objects nil))
    (table-scene-init table-pos-y)
    (setq table-objects (list *table*))
    (robot-scene-init :table-pos *table-pos*)

    (box-scene-init scene-alist :base-frame-id *base-frame-id*)
    (setq table-objects (append table-objects *boxes*))
    (wall-scene-init)
    (if wall-p
      (progn
        (setq table-objects
              (append table-objects
                      (list *left-wall* *front-wall*)))))

    ;: viewer-init
    (send *ri* :objects table-objects)
    (make-irtviewer :title "Robot Viewer")
    (objects (append (list *robot*) table-objects) *irtviewer*)))


(defun pilot-scene-init (&key (table-pos-y 0) (scene-alist nil) (wall-p nil))
  (if (null scene-alist)
    (setq scene-alist (load-scene-alist "one_ctb")))
  (let ((table-objects nil))
    (pilot-table-scene-init table-pos-y)
    (setq table-objects (list *pilot-table*))
    (pilot-robot-scene-init :table-pos *table-pos*)

    (pilot-device-scene-init scene-alist)
    (setq table-objects
          (append table-objects *pilot-devices*))
    (pilot-wall-scene-init)
    (if wall-p
      (progn
        (setq table-objects
              (append table-objects
                      (list *pilot-left-wall* *pilot-front-wall*)))))
    ;: viewer-init
    (setq *pilotviewer* (instance x::irtviewer :create :title "Pilot Viewer"))
    (objects (append (list *pilot*) table-objects) *pilotviewer*)))


(provide :common-scenes "common-scenes.l")
