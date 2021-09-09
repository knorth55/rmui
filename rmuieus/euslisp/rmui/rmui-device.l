(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")
(require :imu-utils "package://rmuieus/euslisp/rmui/imu-utils.l")


(ros::roseus-add-msgs "force_proximity_ros")
(ros::roseus-add-msgs "std_msgs")


(defclass rmui-device
  :super propertied-object
  :slots (tfl
          namespace
          prx-threshold
          prx-position-threshold
          device-frame-id
          world-frame-id
          device
          proximity
          device-prx-vectors
          prx-initial-positions
          prx-x45-positions
          prx-y45-positions
          prx-z45-positions
          prx-initial-vectors
          prx-x45-vectors
          prx-y45-vectors
          prx-z45-vectors
          groupname
          ))


(defmethod rmui-device
  (:init (&key ((:namespace ns))
               ((:prx-threshold prxthre))
               ((:prx-position-threshold prxposthre))
               ((:device-frame-id devfrmid))
               ((:world-frame-id wldfrmid))
               ((:prx-initial-positions prxinitpos) *prx-initial-positions*)
               ((:prx-x45-positions prxx45pos) *prx-x45-positions*)
               ((:prx-y45-positions prxy45pos) *prx-y45-positions*)
               ((:prx-z45-positions prxz45pos) *prx-z45-positions*)
               ((:prx-initial-vectors prxinitvec) *prx-initial-vectors*)
               ((:prx-x45-vectors prxx45vec) *prx-x45-vectors*)
               ((:prx-y45-vectors prxy45vec) *prx-y45-vectors*)
               ((:prx-z45-vectors prxz45vec) *prx-z45-vectors*)
               ((:groupname gpn)))
    (setq namespace (if ns ns ""))
    (setq tfl (instance ros::transform-listener :init))
    (setq device (make-cube 120 120 120))
    (setq prx-threshold prxthre)
    (setq prx-position-threshold prxposthre)
    (setq device-frame-id devfrmid)
    (setq world-frame-id wldfrmid)
    (setq prx-initial-positions prxinitpos)
    (setq prx-x45-positions prxx45pos)
    (setq prx-y45-positions prxy45pos)
    (setq prx-z45-positions prxz45pos)
    (setq prx-initial-vectors prxinitvec)
    (setq prx-x45-vectors prxx45vec)
    (setq prx-y45-vectors prxy45vec)
    (setq prx-z45-vectors prxz45vec)
    (setq device-prx-vectors (copy-prx-vectors prx-initial-vectors))
    (setq groupname (if gpn gpn))
    (if groupname (ros::create-nodehandle groupname))
    (if groupname
      (ros::subscribe (format nil "~A/output/proximities" namespace)
                      force_proximity_ros::ProximityArray
                      #'send self :proximity-cb
                      :groupname groupname)
      (ros::subscribe (format nil "~A/output/proximities" namespace)
                      force_proximity_ros::ProximityArray
                      #'send self :proximity-cb))
    self)
  (:device-prx-vectors () device-prx-vectors)
  (:proximity () proximity)
  (:proximity-cb (msg)
    (setq proximity (mapcar #'(lambda (x) (send x :average)) (send msg :proximities))))
  (:update-device-orientation ()
    (let* ((wait-p (send tfl :wait-for-transform
                         world-frame-id device-frame-id (ros::time 0) 0.1))
           (device-coords
             (if wait-p
               (send tfl :lookup-transform
                     world-frame-id device-frame-id (ros::time 0)))))
      (if device-coords
        (send device :newcoords device-coords)
        (send device :worldcoords))))
  (:get-contact-positions ()
    (let ((prx-positions
            (send self :get-device-prx-positions))
          (prx-index 0)
          (contact-positions nil))
      (if prx-positions
        (dolist (prx proximity)
          (when (> prx prx-threshold)
            (setq contact-positions
                  (append contact-positions
                          (list (elt prx-positions prx-index)))))
          (setq prx-index (+ prx-index 1))))
      contact-positions))
  (:update-device-prx-vectors ()
    (let ((device-rot (send device :copy-rot)))
      (setq device-prx-vectors
            (mapcar #'(lambda (prx-v)
                        (let ((v (copy-seq prx-v))) (transform device-rot v)))
                    prx-initial-vectors))
      t))
  (:get-device-prx-positions ()
    (if (send self :update-device-prx-vectors)
      (let* ((prx-vectors-and-positions
               (send self :select-prx-vectors-and-positions))
             (prx-vectors (car prx-vectors-and-positions))
             (prx-positions (cadr prx-vectors-and-positions)))
        (if (and prx-vectors prx-positions)
          (mapcar #'(lambda (prx-v)
                      (send self :get-prx-position prx-v prx-vectors prx-positions))
                  device-prx-vectors)))))
  (:select-prx-vectors-and-positions ()
    (let* ((device-center-vectors
             (list (elt device-prx-vectors 4) (elt device-prx-vectors 9)
                   (elt device-prx-vectors 14) (elt device-prx-vectors 19)
                   (elt device-prx-vectors 24) (elt device-prx-vectors 29)))
           (diff-x-vs
             (mapcar #'(lambda (v) (norm (v- (float-vector 1.0 0 0) v))) device-center-vectors))
           (diff-x-p (mapcar #'(lambda (v) (< v prx-position-threshold)) diff-x-vs))
           (x-p (not (every #'null diff-x-p)))
           (diff-y-vs
             (mapcar #'(lambda (v) (norm (v- (float-vector 0 1.0 0) v))) device-center-vectors))
           (diff-y-p (mapcar #'(lambda (v) (< v prx-position-threshold)) diff-y-vs))
           (y-p (not (every #'null diff-y-p)))
           (diff-z-vs
             (mapcar #'(lambda (v) (norm (v- (float-vector 0 0 1.0) v))) device-center-vectors))
           (diff-z-p (mapcar #'(lambda (v) (< v prx-position-threshold)) diff-z-vs))
           (z-p (not (every #'null diff-z-p))))
      (cond
        ((and x-p y-p z-p)
         (list prx-initial-vectors prx-initial-positions))
        (x-p (list prx-x45-vectors prx-x45-positions))
        (y-p (list prx-y45-vectors prx-y45-positions))
        (z-p (list prx-z45-vectors prx-z45-positions))
        (t (list nil nil)))))
  (:get-prx-position (prx-v prx-vectors prx-positions)
    (let*
      ((prx-v (scale (/ 1.0 (norm prx-v)) prx-v))
       (diff-vs (mapcar #'(lambda (v) (norm (v- prx-v v))) prx-vectors))
       (diff-p (mapcar #'(lambda (v) (< v prx-position-threshold)) diff-vs))
       diff-v-min prx-index)
      (when (every #'null diff-p) (return-from :get-prx-position nil))
      ;; get diff-p t and smallest diff-v position
      (dotimes (diff-index (length prx-vectors))
        (if (and (elt diff-p diff-index)
                 (or (null diff-v-min)
                     (> diff-v-min (elt diff-vs diff-index))))
          (setq prx-index diff-index
                diff-v-min (elt diff-vs diff-index))))
      (if prx-index (elt prx-positions prx-index)))))


(provide :rmui-device "rmui-device.l")
