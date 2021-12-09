(defun remove-key-from-args (args key)
  (let ((key-position (position key args)))
    (if key-position
      (if (>= (length args) (+ 2 key-position))
        (setq args (subseq args 0 key-position))
        (setq args (append (subseq args 0 key-position)
                         (subseq args (+ 2 key-position))))))
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
