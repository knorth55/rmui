(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "topic_tools")


(defun topic-mux-select (service-name topic-name)
  (if (ros::wait-for-service service-name 1)
    (let ((req (instance topic_tools::MuxSelectRequest :init :topic topic-name))
          (res))
     (setq res (ros::service-call service-name req))
     (if res
       (ros::ros-info
         (format nil "Switch topic mux from ~A to ~A"
                 (send res :prev_topic) topic-name)))
     res)))


(defun start-speech-recognition (start-service-name)
  (if (ros::wait-for-service start-service-name 1)
    (progn
      (ros::ros-info "Start speech recognition")
      (ros::service-call start-service-name
                         (instance std_srvs::EmptyRequest :init)))))


(defun stop-speech-recognition (stop-service-name)
  (if (ros::wait-for-service stop-service-name 1)
    (progn
      (ros::ros-info "Stop speech recognition")
      (ros::service-call stop-service-name
                         (instance std_srvs::EmptyRequest :init)))))


(defun remove-key-from-args (args key)
  (let ((key-position (position key args)))
    (if key-position
      (if (>= (length args) (+ 2 key-position))
        (setq args (copy-object (subseq args 0 key-position)))
        (setq args (append (copy-object (subseq args 0 key-position))
                           (copy-object (subseq args (+ 2 key-position)))))))
    args))


(defun override-key-value-in-args (args key value)
  (let ((key-position (position key args)))
    (if key-position
      (setf (elt args (+ 1 key-position)) value)
      (setq args (append args (list key value))))
    args))


(defun convert-string-to-sym (name)
  (let ((node-sym (read-from-string name)))
    (remove '|n|
            (mapcar #'(lambda (x) (if (listp x) (remove '|n| x) x))
                    node-sym))))


(defun convert-scene-state-to-string
  (org-scene-state &key (skip-wall-contact nil) (skip-object-location nil))
  (if (and org-scene-state (listp org-scene-state))
    (let ((str (make-string-output-stream 128))
          (scene-state (copy-tree org-scene-state)))
      (if (or skip-wall-contact skip-object-location)
        (dotimes (scene-i (length scene-state))
          (let* ((scene-a (copy-tree (elt scene-state scene-i)))
                 (left-wall-contact-state
                   (if skip-wall-contact
                     (assoc :left-wall-contact-state scene-a)))
                 (right-wall-contact-state
                   (if skip-wall-contact
                     (assoc :right-wall-contact-state scene-a)))
                 (front-wall-contact-state
                   (if skip-wall-contact
                     (assoc :front-wall-contact-state scene-a)))
                 (object-location-state
                   (if skip-object-location
                     (assoc :object-location-state scene-a))))
            (if left-wall-contact-state
              (setq scene-a (remove left-wall-contact-state scene-a)))
            (if right-wall-contact-state
              (setq scene-a (remove right-wall-contact-state scene-a)))
            (if front-wall-contact-state
              (setq scene-a (remove front-wall-contact-state scene-a)))
            (if object-location-state
              (setq scene-a (remove object-location-state scene-a)))
            (if (or left-wall-contact-state
                    right-wall-contact-state
                    front-wall-contact-state
                    object-location-state)
              (setf (elt scene-state scene-i) scene-a)))))
      (format str "(")
      (dolist (scst (butlast scene-state))
        (format str "(")
        (dolist (e (butlast scst))
          (format str "~A\\n" e))
        (format str "~A)" (car (last scst)))
        (format str "\\n"))
      (format str "(")
      (dolist (e (butlast (car (last scene-state))))
        (format str "~A\\n" e))
      (format str "~A))" (car (last (car (last scene-state)))))
      (get-output-stream-string str))
    (format nil "~A" scene-state)))


(provide :common-utils "common-utils.l")
