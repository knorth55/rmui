(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")


(defclass statenet-graph
  :super pddl-graph
  :slots ())


(defmethod statenet-graph
  (:init ()
    (send-super :init)
    (send self :current-draw-mode))
  (:add-pddl-state-node
   (pddl-state &key (node-name))
   (let ((nd (send self :search-node-from-pddl-state pddl-state)))
     (unless nd
       (let ((n (instance arced-node :init :name
                          (cond
                           ((eq node-name :simple)
                            (intern (string-upcase (format nil "node_~A" cntr))))
                           ((eq node-name :pprint)
                            (cond
                              ((listp (car pddl-state))
                               (convert-scene-state-to-string pddl-state))
                              (t (format nil "~A" pddl-state))))
                           (t (string-upcase (format nil "~A" pddl-state)))))))
         (incf cntr)
         (setf (get n :pddl-state) pddl-state)
         (send self :add-node n)
         (setq nd n)))
     nd))
  (:add-arc-from-to
   (from to name &key (both nil))
   (if (find-if #'(lambda (x) (and (eq (cdr x) to)
                                   (string-equal (car x) name)))
                (send from :neighbor-action-alist))
     (ros::ros-warn (format nil "[:add-arc-from-to] same arc found: ~A" name))
     (let ((ar (instance costed-arc :init from to 1)))
       (send ar :name name)
       (send from :neighbors))))
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
          ; (draw-both-arc t)
          ; (draw-arc-label t)
          (draw-result-only (not (send self :get :draw-merged-result)))
          )
      (labels ((graphviz-node-name
                (n)
                (cdr (assoc n node-alist))))
        (format strm "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (let* ((target-node-name (send target-node :name))
                 (target-node-sym
                   (if target-node-name (convert-string-to-sym target-node-name))))
            (format strm "   ")
            (format strm "~A [label = \"~A\" ~A];~%"
                    (graphviz-node-name target-node)
                    ;; modify node-name
                    ;; TODO: skip for better pdf
                    (if (listp (car target-node-sym))
                      (convert-scene-state-to-string
                        target-node-sym
                        :skip-wall-contact t
                        :skip-object-location t)
                      target-node-name)
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
          t))))
  (:write-to-dot (fname &optional result-path (title "output"))
   "write graph structure to dot(graphviz) file
Args:
  fname: filename for output
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
"
   (with-open-file
    (f fname :direction :output)
    (send self :write-to-dot-stream f result-path title))
   t))


(provide :statenet-graph "statenet-graph.l")
