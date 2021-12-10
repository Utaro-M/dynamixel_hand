(load "package://dynamixel_hand/euslisp/rhp3hand_r-interface.l")
(load "package://dynamixel_hand/euslisp/rhp3hand_l-interface.l")

(ros::roseus "manus_node")

(defclass my-coords
  :super propertied-object
  :slots ()
  )

(defmethod manus-node
  (:init (&key (realp t) (ns "manus"))
         (setq *rhand* (instance rhp3hand_r-robot :init))
         (setq *lhand* (instance rhp3hand_l-robot :init))
         (when realp
           (setq *rhand-ri* (instance* rhp3hand_r-interface :init nil))
           (setq *lhand-ri* (instance* rhp3hand_l-interface :init nil))
           )
         ;; subscribe finger angle vector from gloves
         (ros::subscribe (format nil "/~A/r_angles" ns) std_msgs::string #'send self :r-angles-cb)
         (ros::subscribe (format nil "/~A/l_angles" ns) std_msgs::string #'send self :l-angles-cb)
         ;; subscribe vector from tactile sensors
         (ros::subscribe (format nil "/~A/r_tactile" ns) std_msgs::string #'send self :r-tactile-cb)
         (ros::subscribe (format nil "/~A/l_tactile" ns)  std_msgs::string #'send self :l-tactile-cb)
         ;; subscribe dynamixel current
         (ros::subscribe (format nil "/~A/dynamixel_state" ns) DynamixelStateList,  #'send self :state_cb)
         ;; publish vibration
         (ros::advertise (format nil "/~A/r_vibration" ns) std_msgs::string 1)
         (ros::advertise (format nil "/~A/l_vibration" ns) std_msgs::string 1)
         self)
  (:r-angles-cb (msg)
                   (cond
                    (((send msg :)
                      (send *rhand* :angle-vector #f(-90 95 30)))
                     (()
                      (send *rhand* :angle-vector #f(-90 95 30)))
                     (()
                      (send *rhand* :angle-vector #f(-90 95 30)))
                     ))
                    (send *rhand-ri* :angle-vector-with-effort (send *rhand* :angle-vector) effort 500)
                    (send *rhand-ri* :wait-interpolation)
                    )
  (:l-angles-cb (msg)
                )
  (:r-tactile-cb (tactile-msg)
                 (setq vibration-msg (instance std_msgs::string :init))
                 (cond
                  (()
                   (send vibration-msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec))))
                  )
                 ;; (ros::ros-info "msg [~A]" (send msg :data))
                 (ros::publish (format nil "/~A/r_vibration" ns) vibration-msg)
                 )
  (:l-tactile-cb (msg)
                 )
  (:state-callback ()
                   )
  
  )

