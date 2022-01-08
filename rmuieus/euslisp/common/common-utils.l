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


(defun convert-scene-state-to-string (scene-state)
  (if (and scene-state (listp scene-state))
    (let ((str (make-string-output-stream 128)))
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
