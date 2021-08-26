(defun load-statenet-state (state)
  (load (format nil "package://rmuieus/euslisp/statenet/statenet-~A-state.l"
                (string-downcase state))))


(provide :statenet-util "statenet-util.l")
