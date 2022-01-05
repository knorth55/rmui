(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")
(require :imu-utils "package://rmuieus/euslisp/rmui/imu-utils.l")


(ros::roseus-add-msgs "force_proximity_ros")
(ros::roseus-add-msgs "std_msgs")


(defclass rmui-device
  :super propertied-object
  :slots (tfl
          namespace
          groupname
          prx-threshold
          prx-position-threshold
          device-frame-id
          device-world-frame-id
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
          ))


(defmethod rmui-device
  (:init (&key ((:namespace ns))
               ((:prx-threshold prxthre))
               ((:prx-position-threshold prxposthre))
               ((:device-frame-id devfrmid))
               ((:device-world-frame-id devwldfrmid))
               ((:prx-initial-positions prxinitpos) *prx-initial-positions*)
               ((:prx-x45-positions prxx45pos) *prx-x45-positions*)
               ((:prx-y45-positions prxy45pos) *prx-y45-positions*)
               ((:prx-z45-positions prxz45pos) *prx-z45-positions*)
               ((:prx-initial-vectors prxinitvec) *prx-initial-vectors*)
               ((:prx-x45-vectors prxx45vec) *prx-x45-vectors*)
               ((:prx-y45-vectors prxy45vec) *prx-y45-vectors*)
               ((:prx-z45-vectors prxz45vec) *prx-z45-vectors*)
               ((:transform-listener tflstner))
               ((:groupname gpn)))
    (setq namespace (if ns ns ""))
    (setq tfl (if tflstner tflstner (instance ros::transform-listener :init)))
    (setq device (make-cube 120 120 120))
    (setq prx-threshold prxthre)
    (setq prx-position-threshold prxposthre)
    (setq device-frame-id devfrmid)
    (setq device-world-frame-id devwldfrmid)
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
                         device-world-frame-id device-frame-id (ros::time 0) 0.1))
           (device-coords
             (if wait-p
               (send tfl :lookup-transform
                     device-world-frame-id device-frame-id (ros::time 0)))))
      (if device-coords
        (progn
          (send device :newcoords device-coords)
          (send device :worldcoords)))))
  (:get-device-state ()
    (if (send self :update-device-prx-vectors)
      (let* ((device-rotation-state
               (send self :get-device-rotation-state))
             (device-prx-positions
               (send self :get-device-prx-positions
                     device-rotation-state))
             (device-contact-positions
               (send self :get-device-contact-positions
                     device-prx-positions)))
        (list (list :rotation-state device-rotation-state)
              (list :contact-positions device-contact-positions)))))
  (:get-device-contact-positions (device-prx-positions)
    (let ((device-prx-index 0)
          (device-contact-positions nil))
      (if device-prx-positions
        (dolist (prx proximity)
          (when (> prx prx-threshold)
            (setq device-contact-positions
                  (append device-contact-positions
                          (list (elt device-prx-positions device-prx-index)))))
          (setq device-prx-index (+ device-prx-index 1))))
      device-contact-positions))
  (:update-device-prx-vectors ()
    (let ((device-rot (send device :copy-rot)))
      (setq device-prx-vectors
            (mapcar #'(lambda (prx-v)
                        (let ((v (copy-seq prx-v))) (transform device-rot v)))
                    prx-initial-vectors))
      t))
  (:get-device-prx-positions (device-rotation-state)
    (let* ((device-rotation-state
             (send self :get-device-rotation-state))
           (prx-vectors
             (send self :select-prx-vectors device-rotation-state))
           (prx-positions
             (send self :select-prx-positions device-rotation-state)))
      (if (and prx-vectors prx-positions)
        (mapcar #'(lambda (prx-v)
                    (send self :get-prx-position prx-v prx-vectors prx-positions))
                device-prx-vectors))))
  (:get-device-rotation-state ()
    (let* ((device-center-vectors
             (list (elt device-prx-vectors 4)
                   (elt device-prx-vectors 9)
                   (elt device-prx-vectors 14)
                   (elt device-prx-vectors 19)
                   (elt device-prx-vectors 24)
                   (elt device-prx-vectors 29)))
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
      (cond ((and x-p y-p z-p) :neutral)
            (x-p :x45)
            (y-p :y45)
            (z-p :z45)
            (t nil))))
  (:select-prx-vectors (device-rotation-state)
    (cond
      ((equal device-rotation-state :neutral) prx-initial-vectors)
      ((equal device-rotation-state :x45) prx-x45-vectors)
      ((equal device-rotation-state :y45) prx-y45-vectors)
      ((equal device-rotation-state :z45) prx-z45-vectors)
      (t nil)))
  (:select-prx-positions (device-rotation-state)
    (cond
      ((equal device-rotation-state :neutral) prx-initial-positions)
      ((equal device-rotation-state :x45) prx-x45-positions)
      ((equal device-rotation-state :y45) prx-y45-positions)
      ((equal device-rotation-state :z45) prx-z45-positions)
      (t nil)))
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
      (if prx-index (elt prx-positions prx-index))))
  (:spin-once ()
    (if groupname
      (ros::spin-once groupname)
      (ros::spin-once))))


(provide :rmui-device "rmui-device.l")
