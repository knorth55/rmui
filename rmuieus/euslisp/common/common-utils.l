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


(provide :common-utils "common-utils.l")
