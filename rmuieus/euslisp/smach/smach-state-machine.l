(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")


(defmethod state-machine
  (:bottom-active-state (&optional top-active-state)
    (if (null top-active-state)
      (setq top-active-state active-state))
    (if (and top-active-state (listp top-active-state))
      (flatten
        (mapcar #'(lambda (x) (send self :bottom-active-state x))
                top-active-state))
      (if (and top-active-state (send top-active-state :submachine))
        (let ((sub-active-state
                (send (send top-active-state :submachine) :bottom-active-state)))
          (if sub-active-state
            sub-active-state top-active-state))
        top-active-state)))
  (:append-goal-state (gs)
    (if (atom gs) (setq gs (list gs)))
    (setq gs (mapcar #'(lambda(g) (instance state :init g nil)) gs))
    (dolist (n gs) (send self :add-node n))
    (send-super :goal-state (append goal-state gs)))
  (:write-to-dot-stream (&optional (strm *standard-output*) result-path (title "output"))
   "write graph structure to stream as dot(graphviz) style
Args:
  strm: stream class for output
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
"
    (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
           (mapcar #'(lambda (n)
                       (cons n (string (gensym))))
                   nodes))
          (draw-both-arc (send self :get :draw-both-arc))
          (draw-arc-label (send self :get :draw-arc-label))
          (draw-result-only (not (send self :get :draw-merged-result)))
          )
      (labels ((graphviz-node-name
                (n)
                (cdr (assoc n node-alist))))
        (format strm "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (let* ((target-node-name (send target-node :name))
                 (target-node-sym (if target-node-name (read-from-string target-node-name))))
            (format strm "   ")
            (format strm "~A [label = \"~A\" ~A];~%"
                    (graphviz-node-name target-node)
                    ;; modify node-name
                    (cond
                      ((and (or (equal :action-node (send target-node :get :node-type))
                                (equal :teach-node (send target-node :get :node-type)))
                            (listp target-node-sym))
                       (format nil "~A" (car target-node-sym)))
                      ((and (equal :state-node (send target-node :get :node-type))
                            (listp target-node-sym)
                            (listp (car target-node-sym)))
                       ;; TODO: skip for better pdf
                       (convert-scene-state-to-string
                         target-node-sym
                         :skip-wall-contact t
                         :skip-object-location t))
                      (t target-node-name))
                    (if (send target-node :image)
                        (format nil ", image=\"~A\", imagescale=true" (send target-node :image))
                        ""))))
        ;; first of all, write result-path
        (let ((result-tbl (make-hash-table :test #'equal))
              (draw-tbl (make-hash-table :test #'equal))
              )
          ;; write result-path
          (let (anode)
            (while (and (setq anode (pop result-path)) result-path)
              (setf (gethash (cons (send anode :state)
                                   (send (car result-path) :state))
                             result-tbl)
                    (list "color = red"))
              ))
          ;; write arcs
          (dolist (target-node nodes)
            (let ((neighbors
                   (send self :successors target-node)
                   ))
              (dolist (neighbor neighbors)
                (let ((narc (car neighbor))
                      (nnode (cdr neighbor))
                      (draw-arc t) result
                      options)
                  ;; Is there in result path ?
                  (when (and (not draw-result-only)
                             (gethash (cons target-node nnode) result-tbl))
                    (setq result t)
                    (push "color = red" options))
                  (when draw-arc-label
                    (push (format nil "label = \"~A\"" (send narc :name)) options))
                  ;; checked drawed for both direction
                  (unless draw-both-arc
                    (cond
                     ((gethash (cons target-node nnode) draw-tbl)
                      (setq draw-arc nil)
                      )
                     ((gethash (cons nnode target-node) draw-tbl)
                      (setq draw-arc nil)
                      )
                     ((member target-node
                              (mapcar #'cdr (send self :successors nnode)))
                      (push "dir = both" options)
                      )
                     ))
                  (unless options (setq options :no-option))
                  (when draw-arc
                    (setf (gethash (cons target-node nnode) draw-tbl)
                          options))
                  ))
              ))
          (flet ((print-arcs (tbl)
                  (maphash
                   #'(lambda (key options)
                       (format strm "   ")
                       (format strm "~A -> ~A"
                               (graphviz-node-name (car key))
                               (graphviz-node-name (cdr key)))
                       (if (eq options :no-option)
                           (format strm ";~%")
                         (progn
                           (format strm "[")
                           (dolist (opt options)
                             (unless (eq opt (car options))
                               (format strm ", "))
                             (format strm "~A" opt))
                           (format strm "];~%"))))
                   tbl)))
            (when draw-result-only (print-arcs result-tbl)) ;; draw-resut
            (print-arcs draw-tbl)
            )
          (format strm "}~%")
          t)))))


(provide :smach-state-machine "smach-state-machine.l")
