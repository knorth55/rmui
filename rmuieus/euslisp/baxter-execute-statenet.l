#!/usr/bin/env roseus

(require :common-statenet-client "package://rmuieus/euslisp/common/common-statenet-client.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")

(ros::roseus-add-msgs "rmui_msgs")


(defun main (state &key (n-box nil) (server-name "/statenet_server"))
  (let ((speech-server-name "/speech_recognition")
        (res))
    (ros::roseus "baxter_execute_statenet")
    (stop-speech-recognition (format nil "~A/stop" speech-server-name))
    (topic-mux-select "/speech_to_text_mux/select" "/speech_to_text_other")
    (setq res (statenet-action-client-main state :n-box n-box :server-name server-name))
    (start-speech-recognition (format nil "~A/start" speech-server-name))
    (topic-mux-select "/speech_to_text_mux/select" "/speech_to_text_google")
    res))


(defun signal-hook (sig code)
  (start-speech-recognition "/speech_recognition/start")
  (topic-mux-select "/speech_to_text_mux/select" "/speech_to_text_google")
  (reset))


(unix:signal 2 'signal-hook)
(unix:signal 9 'signal-hook)
(unix:signal 15 'signal-hook)
