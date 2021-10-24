;;(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(load "package://pr2eus/robot-interface.l")
(require :rhp3hand_r "package://dynamixel_hand/models/rhp3hand_r.l")
(ros::load-ros-manifest "control_msgs")

(defclass rhp3hand_r-interface
  :super robot-interface;;rtm-ros-robot-interface
  :slots ())
(defmethod rhp3hand_r-interface
  (:init
    (&rest args &key ((:controller-timeout ct) 5))
    (prog1
      (send-super* :init :robot rhp3hand_r-robot :controller-timeout ct args)
      )
    )
  (:default-controller
    ()
    (list
      (list
        (cons :controller-action "right_hand_controller/follow_joint_trajectory_action")
        (cons :controller-state "right_hand_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))
        )
      )
    )
  ;; (:current-vector-simulation
  ;;  (av tm ctype)
  ;;  (let* ((prev-av (send robot :angle-vector)))
  ;;    (send-all (gethash ctype controller-table) :push-angle-vector-simulation av tm prev-av)))
  (:current-vector
   (av cv &optional (tm 1000) (ctype controller-type) (start-time 0) &key (scale 1))
   "Send joint angle to robot, this method returns immediately, so use :wait-interpolation to block until the motion stops.
- cv : current vector [deg]
   if designated tm is faster than fastest speed, use fastest speed
   if not specified, it will use 1/scale of the fastest speed .
   if :fast is specified use fastest speed calculated from max speed
- start-time : time to start moving
- scale : if tm is not specified, it will use 1/scale of the fastest speed
"
   (format t "send current vector ~A" cv)
   ;; for simulation mode
   ;; (when (send self :simulation-modep)
   ;;   (if cv (send self :current-vector-simulation cv tm ctype)))
   ;; (let ((trajpoint (list (instantiate float-vector (length cv));; positions
   (let ((trajpoint (list av ;; positions
                 (instantiate float-vector (length cv));; velocities
                 (/ tm 1000.0);; duration
                 (instantiate float-vector (length cv));; accelerations
                 cv));; current vector
         (joint-names (cdr (assoc :joint-names (car (send self ctype))))))
     (format t "(assoc :joint-names (send self ctype)) =~A~%" (car (send self ctype)))
     (format t "(assoc :joint-names (send self ctype)) =~A~%" (assoc :joint-names (car (send self ctype))))
     ;;model
     ;; !!!     (send robot :angle-vector av)

     ;; (mapcar
       ;;  #'(lambda (action param)
       ;;      (send self :send-ros-controller
       ;;            action (cdr (assoc :joint-names param)) ;; action server and joint-names
       ;;            start-time  ;; start time
       ;;            trajpoints))
        ;; caction (send self ctype))))
  ;; (send self :send-ros-controller
  ;;       caction (cdr (assoc :joint-names (send self ctype))) ;; action server and joint-names
  ;;       start-time  ;; start time
  ;;       trajpoint)
  ;; av)
  ;; (:send-ros-controller
     ;;  (action joint-names starttime trajpoints)

   ;;simulation
   ;; (when (send self :simulation-modep)
     ;;   (return-from :send-ros-controller nil))
     (format t "joint-names = ~A~%" joint-names)
   (dolist (name joint-names)
     (unless (send robot :joint name)
       (warning-message 1 "[robot-interface.l] (send-ros-controller) could not find joint-name '~A' (~A)~%" name (send robot :joint name))
       (return-from :current-vector nil)))
   (format t "ctype = ~A~%" ctype)
   (format t "controller-table = ~A~%" controller-table)
   (let* ((caction (car (gethash ctype controller-table)))
          ;; (caction (cdr (assoc :action-type (car (send self ctype)))))
          (goal (send caction :make-goal-instance))
	  (goal-points nil)
	  (st (if (numberp start-time)
                  (ros::time+ (ros::time-now) (ros::time start-time))
                start-time))
	  (joints (mapcar #'(lambda (x)
                          (send robot (intern (string-upcase x) *keyword-package*)))
                      joint-names)))
     (format t "caction = ~A~%" caction)
     (send goal :header :seq 1)
     (send goal :header :stamp st)

     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp st)
     (let* ((all-positions (elt trajpoint 0))
            (all-velocities (elt trajpoint 1))
            (duration (elt trajpoint 2))
            (all-accelerations (elt trajpoint 3))
            (all-efforts (elt trajpoint 4))
            (positions (instantiate float-vector (length joint-names)))
            (velocities (instantiate float-vector (length joint-names)))
            (accelerations (instantiate float-vector (length joint-names)))
            (efforts (instantiate float-vector (length joint-names))))
       (dotimes (i (length joints))
         (let* ((joint (elt joints i))
                (id (position joint (send robot :joint-list)))
                p v a e)
           (setq p (elt all-positions id)
                 v (elt all-velocities id)
                 a (elt all-accelerations id)
                 e (elt all-efforts id))
           (cond
            ((derivedp joint rotational-joint)
             (setq p (deg2rad p))
             (setq v (deg2rad v))
             (setq a (deg2rad a))
             ;; (setq e (deg2rad e))
             )
            (t
             (setq p (* 0.001 p))
             (setq v (* 0.001 v))
             (setq a (* 0.001 a))
             ;; (setq e (* 0.001 e))
             ))
             (setf (elt positions i) p)
             (setf (elt velocities i) v)
             (setf (elt accelerations i) a)
             (setf (elt efforts i) e)))
         (push (instance trajectory_msgs::JointTrajectoryPoint
                         :init
                         :positions positions
                         ;; :velocities velocities
                         ;; :accelerations accelerations
                         :effort efforts
                         :time_from_start (ros::time duration))
               goal-points)
	   )
       (send self :spin-once)
       (send goal :goal :trajectory :points goal-points)
     (send caction :send-goal goal)
     )))
  )

(defun rhp3hand_r-init (&rest args)
  (if (not (boundp '*ri*))
    (setq *ri* (instance* rhp3hand_r-interface :init args)))
  (if (not (boundp '*rhp3hand_r*))
    (setq *rhp3hand_r* (instance rhp3hand_r-robot :init)))
  )
