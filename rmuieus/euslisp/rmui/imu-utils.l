(defun update-device-orientation ()
  (unless (boundp '*tfl*)
    (return-from update-device-orientation nil))
  (let* ((wait-p (send *tfl* :wait-for-transform
                       *world-frame-id* *device-frame-id* (ros::time 0) 0.1))
         (device-coords
           (if wait-p
             (send *tfl* :lookup-transform
                   *world-frame-id* *device-frame-id* (ros::time 0)))))
    (if device-coords
      (send *device* :newcoords device-coords)
      (send *device* :worldcoords))))


(provide :imu-utils "package://rmuieus/euslisp/imu-utils.l")
