^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmuieus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2022-05-09)
------------------
* make motions faster
* use rmui0 instead of rmui2
* update gitignore
* update scene
* Contributors: Shingo Kitagawa

2.1.0 (2022-02-08)
------------------
* refactor
* update timeout and length
* fix conditions
* add sensitive
* add launch args
* add grasp mask rcnn key
* update statenet server
* support initial pileup state
* add grasp pileup scene
* support toyobject
* ignore rotation of object
* fix typo
* fix typo
* refactor
* save box-type in statenet-server
* store box-types
* check contact surface freq
* update symbol length
* update condition
* update box shape
* use larm for tidy up
* fix typo in commom-rmui-planner
* fix conditions
* fix grasp approach length
* update scene
* use worldcoords
* add new scene
* add arc cardboard box
* compare z axis too
* move reset pose before recognition
* add if for draw-objects
* add box-shapes var
* update tidyup
* do not update contact positions when it is not recognized
* add timeout for proximity message update
* fix tabletopbox z calculation
* add y_aligned_stuffed_animal
* update z-offset
* fix baxter offset
* add x_aligned_stuffed animal
* update baxter-scenes
* set offset for make-cube
* update scene rpy
* update pilot device rpy
* add comment
* set reset-state z rotation from scene file
* rename func and move
* use (:reset)
* add reset
* use reset-state
* add stuffed animal
* add new scene
* correct box rpy from given rpy
* use toybox when large object not found
* add x_aligned_large_objects
* add largeobject
* fix check with :teach
* skip wall contact and object location for better pdf
* add skip option for better pdf
* move convert-string-to-sym
* support different number object graph merge
* use copy-tree for subseq
* update lift-down-motion
* use tabletop object detector
* longer release distance
* set larger collision bbox
* refactor common-motions
* ungrasp when release
* use cube-rpy for grasp motion
* liftup with reset-rpy
* fix cpi :z
* sort by xy position
* wait for longer time
* remove debug message
* update tidyup scene
* change grasp condition
* wait-for-publisher for recognize
* support :object to recognize
* set base-frame-id var
* use simulation-modep
* wait for publishers
* fix timeout
* use same tfl
* add recognize tabletop objects
* fix grasp motions
* fix typo in some
* fix grasp motions
* fix typo
* rename to tidyup
* add visualize-groupname
* add new task
* fix prev-av
* fix conditions
* refactor bottom cube contact assert
* add dissoc for liftup
* update comment
* disable top push
* assoc cubes when piled up
* add assoc/dissoc two cubes
* fix conditions
* add move in liftup
* fix typo
* fix typo
* add grasp pileup
* add grasp-down action
* add grasp-up
* add grasp release motion
* euslint
* set grasp-arm for both arm
* fix typo
* add top-grasp
* set boxtype and device-type
* add grasp motion
* add grasp coords for human
* add wait for softhand
* grasp in no simulation
* add grasp-coords in scenes
* rename functions
* add grasp-cube and ungrasp-cube
* add grasp conditions
* add grasp action
* add get-grasp-coords
* change device shape in pilot viewer
* update scene functions
* remove unused conditions
* add tidy_up scene
* fix typo
* refactor launch
* remove unused launch
* rename to scene
* add scene config file
* add load-config
* add config
* use baxter + softhand
* change default args
* change default arg
* Contributors: Shingo Kitagawa

2.0.0 (2021-12-27)
------------------
* do not ask teach again
* fix ros-index-choose
* fix typo
* run mux after starting
* fix indent
* pass n_box as topic
* refactor smach-state-machine.l
* return-from functions when input is nil
* check all update device
* fix typo
* add comment
* change release ik step
* fix common-graph
* refactor
* solve ik in release-distance
* skip target-node-name if nil
* set default
* update apps
* update statenet server
* show user ask sentence in rviz
* add collision for push release
* change release distance
* sleep after collision service call
* update jetson box
* update parameters
* add all cube collision
* change hold-type
* add cube for collision in push motion
* change approach distance
* use raw for rotate cube motion
* make collision box smaller
* speak when task is finished
* speak wait interrupt
* fix common-action name
* mkdir when dir does not exists
* add conditions for statenet
* skip interruption in last-state-node and first-state-node
* update add node-type
* change interrupt timeout
* update baxter-scenes
* ask teachin first
* refactor interactor
* check node-type
* remove debug line
* use top active node-type
* fix bottom-active-state
* remove unnecessary eval
* ask human with sound
* update motions
* wait until param
* fix generate-teach smach
* add toggle-server for get-user-response
* refactor statenet-server
* add (:teach) state for teaching
* use topic speech recognition
* fix typo
* use ros-topic
* rename variable
* add interactor with service
* fix generate-teach-smach
* start teachin when planning failed
* add get-teach-node-name
* refactor common-statenet-server
* add top-approach state
* use copy-tree
* updte goal-state when updated
* refactor interactor
* add new statenet task
* fix common-motions
* add top approach
* fix rotate motion
* add dualarm-front/back-left-front/back-right-push-hold
* add dualarm-hold-release
* add dualarm-left-right-push-hold
* fix common-motions
* last-failed last-executed action
* reset executor
* return nil
* fix common-motions
* fix common-motions
* add interrupt in device wait
* add reset-interactor
* update interactor
* refactor
* add :finish
* add next-node-name
* add state goal nodes
* use :finish
* update state state-machine
* fix typo in common-graph.l
* fix typo
* update duration
* finish task
* add ask-user-teach-decision
* add ask user device, finish teach, and finish task
* refactor methods in statenet
* no duration wait
* fix typo
* fix comment
* euslint
* fix common-statenet-server
* fix teach update
* add bottom-active-state
* use lamda-closure
* change optional key
* print dump dirnames
* update common-io
* save executed collabo path in teach
* add additional node-type
* add no-loop in convert-collabo-path-to-state-machine
* add aborts in action
* refactor common-graph
* refactor common-statenet-server
* fix common-statenet-server
* fix common-rmui-updater
* add last-executed-action methods
* add append-executed-collabo-path
* add plan-state-machine-with-trained-paths
* refactor
* update teach-state
* add reset-updater
* reset-executor
* go back to teach state when teach replan failed
* update common-actions
* add push-release-and-reset-motion
* add vw in reset-cube-manip-motion
* update statenet in teach
* fix dualarm-push-release-action
* add method in common-rmui-updater
* set current-draw-mode
* refactor smach graph pdf and add smach-state-machine.l
* pass statenet-graph to planner
* add todo
* remove init from merged statenet
* load statenet in collabo
* add teach functions
* add reset-motion-symbol-queues
* update parameters
* rename variables
* split execute-motion-symbols
* add teach-start-state in userdata
* fix common-statenet-interactor
* fix typo
* update statenet-server
* update common motions
* update baxter-scenes
* fix hold-lift-down motion
* fix pr2-motins
* euslint
* use raw ik in baxter for speed up
* return executed in common-actions
* move pilot only when robot executed
* fix typo in push-motion-step
* add arguments
* add common-collabo-path.l
* update condition in common-statenset-server
* refactor statenet pdf
* add wall args
* add wall-p key
* update state
* add wall contact state
* update motion symbol length
* fix wall-p
* set wall-p nil
* refactor hold motion
* update baxter scenes
* fix common-scenes
* fix typo
* add align_axis args
* add align-axis key
* add n_box args
* add align axis
* add new box type
* update cube motion
* add even case
* commentout collision avoidance
* fix push motion
* add cube collision object method
* add object-id
* add base-frame-id
* euslint
* update baxter reset-manip pose
* add attached object for gripper
* return t
* add get-arm-controller
* fix baxter contact coords
* remove listp
* fix conditions
* set priority for wall motion
* fix conditions
* update scenes
* add wall release motion
* euslint
* update output
* update conditions
* update action names
* refactor solve-ik-rotation-relax
* add motion/action wall next to
* add wall contact state
* add wall limit
* set wall limit parameters
* set table-height
* add wall scene
* add common-utils.l
* fix typo in pr2-motions.l
* fix typo
* if func starts with send eval else not
* remove unused slot
* refactor common-graph
* simplify collabo path
* fix ask function
* update convert graph scripts
* fix action-func-sym
* fix simplify collabo path
* refactor skip interaction
* not teach submachine when skip interaction
* support collabo-path for multiple cubes
* use :pu and :get
* fix typo
* set output screen
* fix cons userdata
* disable collabo training
* add nextto state
* remove comments
* support multiple cube for statenet server
* fix typo
* change args in hold-release-motion
* fix args in common-motions.l
* add use-torso move-robot slots
* update comment
* add collabo arg in statenet server launch
* add skip-interaction key
* add statenet-action-client main
* update statenet state
* update action and action-names
* update statenet state
* support multi cube statenet
* add multi statenet_server launch
* rename function
* merge common-rmui-planninga and common-rmui-server
* change threshold
* fix typo common-actions
* fix typo
* fix next-to motion and action
* update next-to motion and action
* fix typo in common-motions
* update conditions
* fix typo
* return only when executed
* add push-next-to action and motion
* fix typo
* format common-rmui-server
* fix hold-pile-up motion action
* fix typo
* update multi cube pile up
* remove todo
* fix typo
* fix yaml
* add release condition for bottom cube contact
* add lift-up from pile-up
* support used case in planner
* fix typo in common-rmui-executor
* fix typo in hold-pile-up motion
* implement hold-pile-up-motion
* set used when arms contacts other box
* fix eval bug
* support multi cube demo in get-motion-symbols
* add action args in motion symbol
* fix hold-release-motion
* add dualarm-motion-init
* fix motion-init args
* update push conditions
* fix top push
* add get-device-state in rmui-device
* add box_type arg
* refactor to support multiple devices
* contact-positions -> contact-position
* rename functions
* euslint
* update planning
* support multiple device: motion-init
* update device position
* fix comment
* add ri in start/stop grasp
* fix typo
* add multiple pilot device
* show boxes
* use slot variables
* refactor scene functions
* add multi planning launch
* get multi device contact positions
* fix typo in rmui-device
* support multiple device in common-statenet-server
* add devices and namespaces
* update todo comments
* add arg rmui-names
* add :rmui-names key
* show pilot first
* fix typo
* fix use-torso ik
* add table arg
* check if ri is not nil
* add common-rmui-planning
* add use-torso and move-robot
* add use-torso in get-contact-link-list
* add comment
* update pilot end coords
* support human viewer
* update motion-init in pddl-util
* fix typo
* add pilot-action-init
* add vw arg in common-motions.l
* use apply in common-actions.l
* euslint
* change cube -> box
* add cube args in motion and action
* update motion-init
* update common-rmui-executor
* draw pilot viewers
* add robot and ri arg in motion and action
* add pilot scene
* refactor scenes
* use require
* set title for Viewer
* fix typo
* change world frame id
* add rmui_name
* add rmui-name
* add user interrupt in statenet
* add ask-user-interrupt
* update let name
* return timeout when timeout
* fix typo
* add duration in smach node
* add ros-warn
* save task is new or not
* refactor convert-solution-to-smach
* add :convert-collabo-path-to-state-machine
* fix simplify-collabo-path method
* add convert-trans-alist-to-smach and convert-solution-to-trans-alist
* refactor: add space
* add convert-solution-to-state-machine
* add statenet-dualarm-init-state
* special case for init
* add simplify method
* change action name
* sort by directory name
* update stamp
* fix typo
* refactor indent
* store end stamp
* update ros::rate
* dump collabo path
* rename to collabo path and add load functions
* update default param
* update .gitignore
* refactor timeout
* refactor
* refactor common-statenet-server
* use statenet-interactor
* add statenet-interactor
* fix state machine hz
* refactor rmui_statenet_server.launch
* add rqt_ez_publisher
* add ask ros mode
* save executed-state-actions in same dir
* add execute-stamp key
* save executed-state-actions with stamp
* fix ask-userinput
* dump executed-state-actions
* update .gitignore
* refactor
* rename abort and goal state name
* add-goal-nodes
* rename state names
* dump files when replan succeeded
* run rm when exists
* dump collabo statenet in common-statenet-server.l
* update .gitignore
* add collabo-statenet-dir
* add data-dir for dump and load functions
* add object_statenet path
* update common-statenet-server
* format
* set teach-start-state
* fix add-transition
* add next-in-statenet-state
* add-transition
* fix next node append
* fix smach viewer bug
* add show-pdf in statenet-server
* add get-statenet-teach-submachine
* rename function
* add get-statenet-state-submachine
* split state-state-func into two
* change the order of sm publish-structure
* add :state-machine
* refactor
* check if func is lambda-closure or not
* use cons for used-decision
* support sub statemachine
* fix typo
* refactor common-statenet-server
* refactor
* use spin-once
* add spin-once with device-groupname
* add groupname in common-rmui-server
* add rmui-device and rmui-planner
* add spin-once with groupname
* add groupname in rmui-device
* use if
* add rmui updater
* refactor common-rmui-server
* update formating
* use contact-states for communication
* split common-rmui-planner to three files
* fix typo
* add common-statenet-planner
* rename to common-rmui-planner.l
* refactor common-statenet-server
* support namespace nil
* fix typo
* rename files
* make class for rmui-device and rmui-planners
* add dualarm-push-release-action
* move get-iso-stamp
* solver -> slvr
* add comments
* add plan-state-machine method
* add :teach-node in common-statenet-server
* teach-func support
* refactor common-graph
* refactor common-statenet.l
* add reset-state
* refactor ask-user-decision
* use timer for execute-cb
* move main in separate files
* add statenet-action-client
* make common-statenet-server as object
* refactor ask functions
* add ask-user-index-choose
* add use-default
* change tagbody
* add :teach
* add new ask functions
* fix ask-user-abortion
* refactor ros-index-choose
* filter next-sm-actions
* add todo comment
* update replan, next algorithm and add pass
* no-loop fo first solution
* euslint
* implement replan
* add no-loop arg in convert-solution-to-smach
* implement wait user input function
* return final_state in statenet server
* add rmui_msgs as build_depend
* return if aborted or not
* add aborts state
* refactor convert-graph-to-statenet
* resolve name conflicts
* add state-func in common-graph
* add execute-reset-statenet
* refactor server
* move reset-state
* add get-current-state
* add current_state publish
* add StatenetState msgs
* load statenet-util in common-statenet
* fix typo
* rename launch
* rename to statenet server
* add common-statenet-server and client
* set nil for start and goal state
* update comment
* change start-state and goal-state
* add-state-node and add-action-node
* refactor common-statnet
* refactor common-statenet.l
* add server-name key
* override add-arc-from-to
* save merged statenet graph
* add pddl problem
* rename file
* fix typo
* add more statenet state
* euslint
* rename: pddl-common -> pddl-util
* add pr2/baxter-execute-statenet
* add call-execute-statenet
* add statenet action server
* use ros::roseus-add-msgs
* refactor common-statenet.l
* rename files
* solve from statenet graph
* add load-training-statenet-graph
* add convert-solution-to-smach
* remove unused line
* change to use merged graph
* add merge-statenet-graphs
* load common-io in common-graph
* do not use graph
* move add-action-state-in-graph in common-graph
* add load-all functions
* add path key in load/dump functions
* refactor dump functions
* do not set in load functions
* add get-dumpdir-list
* add link-latest-dump-dir
* euslint
* save statenet in planners
* add statenet-graph.l
* rename files
* remove unused line
* remove pprint
* add state and action node in smach
* add common-graph and common-io
* euslint
* add convert-graph-to-smach
* refactor common-planners
* add pdf flag
* fix typo in pr2_rmui_trained_execute.launch
* change function name
* add initial-state
* add rotation state
* add stamp arg in trained execute launch
* fix typo in common-planners.l
* add timestamp in training save dir
* update common-planners.l
* add trained execute launch
* add rest args
* add init state at the top
* euslint
* add pr2/baxter-trained-execute
* return t in common actions
* save pddl-graph in rmui
* refactor common-planners
* refactor common-planners.l
* add scene-states.l in .gitignore
* save scene-states
* refactor common-planners
* move .gitignore
* save training-data in execution
* add training_data
* add *executed-actions*
* rename pddl euslisp files
* refactor common-conditions.l
* evaluate motion symbol to execute actions
* refactor pddl state
* remove return-from from common conditions
* move conditions
* change default problem
* add pddl problems
* ad baxter/pr2 rmui pddl launch
* refactor pddl rmui euslisp codes
* refactor prx-utils.l
* get smallest diff position
* move parameters
* use exec-state-machine
* refactor common-actions.l
* use common-actions in conditions
* fix format
* add main arg in launch
* add pddl and ffha in package.xml
* move pddl execution files
* add rmui-pddl-actions.l
* euslint
* add pr2 and baxter solve_dualarm_liftup
* use common-actions.l in solve-dualarm-liftup.l
* rename solve-rmui to solve-dualarm-liftup.l
* add common-actions.l
* refactor solve-rmui.l
* load common-motions in common-planners
* use smach for execution
* refactor code
* add zrotate in pddl
* add side
* add failed nodes
* add simple pddl
* euslint
* refactor common-planners.l
* remove unused shebang
* add common-conditions.l
* split contact state conditions
* split execute_motion_symbols conditions
* rename to get_motion_symbols
* move to get_motion_symbol
* split into conditions file
* refactor common-motions.l
* remove unused comment out
* Contributors: Shingo Kitagawa

1.0.3 (2021-08-07)
------------------

1.0.2 (2021-07-06)
------------------
* update baxter background
* update waiting command
* update params
* update prx-threshold
* add moving and waiting signal
* update rmui motion symbol length
* fix condition for push-hold-down
* remove comment
* update hold-down conditions
* update baxter offset
* format
* load collision-object-publisher
* add moveit collision object
* add desk-pos and cube-pos
* update pr2-scenes.l
* upset baxter-scenes.l
* euslint
* add get-contact-ik-args, get-contact-cube-coords to make code shorter
* euslint
* add larm/rarm-contact-coords in kinematics simulator
* add link-list in ik
* fix typo in ik rotation-axis
* add larm/rarm-contact-coords
* update pr2 end coordinates
* update baxter end coordinates
* update baxter scene
* fix common-motions
* remove unused function
* add baxter/pr2-dualarm-motions.l
* update baxter-scenes
* format common-motions.l
* fix scenes
* add baxter rmui dummy and baxter rmui
* add baxter-rmui-main.l and baxter-motions.l
* add common-main
* add common-motions.l
* do not use pr2-planners.l
* add euslisp/common
* add rmui-scene.l
* make directories for rmuieus codes
* Contributors: Shingo Kitagawa

1.0.1 (2021-06-16)
------------------
* fix for kinematics simulator
* rename to pr2-rmui-main.l
* Contributors: Shingo Kitagawa

1.0.0 (2021-06-06)
------------------
* update pr2-motions.l
* update pr2-scene
* open grasp
* support number hold-type
* update z direction push
* update scne
* reset cube-manip-pose
* rotate side-push
* add side-push-rotate
* add comment out
* add object-location-state
* lift up in side
* add push-release conditions
* add support-hold-down motion
* add conditions for move-push
* update left/right-push condition
* add push-release conditions for bottom side contact
* refactor conditions
* add support-hold-up
* add bottom-left/right-side-push
* fix typo
* add left-move-push and right-move-push
* fix condition of rotate in x axis
* update get-rotate-height
* add lift-push-rotate
* us different cube
* lift higher
* update cube size
* add reset-cube-manip-motion
* refactor pr2 motions
* fix for real robot
* refactor get-cube-height
* refactor codes
* add support rotate
* refactor conditions
* add condition bottom
* refactor
* use get-cube-height/depth/width
* fix rotate x
* add top/bottom-left/right-push
* add front-left/right-push and back-left/right-push
* remove comment
* add release for rotate
* refactor cube-rpy
* fix typo
* fix rotate x45
* fix indent
* fix rotate-motion conditions
* add t nil in cond
* add rotate x45
* refactor conditions
* refactor comment out
* use eval for conditions
* add push rotate z45
* remove unnecessary conditions
* add comment
* use released
* add rotate motion symbols
* fix typo in pr2-planners.l
* remove unnecessary and
* fix comment out
* add top-push
* update comment
* euslint
* update bottom conditions for rotation
* update viewers
* refactor
* rename device to object
* add x45,y45,z45 states
* remove unused lines
* fix var name
* fix typo
* add push-move-motion
* add assoc/dissoc in rmui-planners.l
* add cube-pos-y arg
* set cube at the corner
* update todo comment
* change planner algorithm
* euslint
* use top and bottom
* refactor codes
* rename device-state to device-contact-state
* chmod -x
* return t
* add imu-utils
* add update-device-state
* add assoc-cube and dissoc-cube in pr2-planner.l
* write cube-centric lift-up and lift-down codes
* rename to contact and discontact
* rotate cube in kinematics simulator
* add object id
* add copy-object
* add +x pr2-scene.l
* add pr2-scene.l
* add *desk*
* fix defvar
* add require in rmui-planners
* Merge pull request `#7 <https://github.com/knorth55/rmui/issues/7>`_ from knorth55/pr2-demo
* fix typo
* update get-motion-symbol and execute-motion-symbol
* add rmui-main and rmui-planners
* add prx-utils
* fix planners
* update pr2-motions
* fix dualarm-switch-rotate
* fix rotate-motion
* add approach-arm in push-motion
* fix switch-rotate-motion
* refactor support-rotate-motion
* add dualarm-switch-rotate
* add get-switch-rotate-angle
* set *table-z*
* add dualarm-support-rotate
* refactor dualarm-push-rotate
* remove unused args
* refactor
* add support-hold-motion
* rename functions
* refactor pr2-motions
* fix bug in get-rotate-height
* add push rotate and support rotate
* add use-torso args
* rename function
* add get-cube-coords
* refactor
* refactor pr2-motions
* remove assoc/dissoc
* update return values
* refactor
* add hold-type
* support rotate-motion in y-axis
* refactor pr2-motions.l
* update rotate-motion to rotate correctly
* add prev-list-coords
* update push hold for rotate
* update push-hold-release motion
* update rotate-motion
* update pr2-motions.l
* return state
* update motions
* update pr2 demos to work correctly
* rename to pr2-planners.l
* support dualarm motions
* refactor motions
* add comment
* add pr2-motion-planner.l
* upda dualarm-hold-push-side
* fix typo in package.xml
* add pr2-motions.l
* add rmuieus
* Contributors: Shingo Kitagawa

0.0.0 (2020-08-15)
------------------
