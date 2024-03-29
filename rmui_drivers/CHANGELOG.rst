^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmui_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.1 (2023-04-02)
------------------

2.3.0 (2022-08-15)
------------------
* update machine name
* Contributors: Shingo Kitagawa

2.2.0 (2022-05-09)
------------------
* update rviz
* fix typo
* add sample launch dir
* change threshold
* add rmui0 to rmui1 gui
* Contributors: Shingo Kitagawa

2.1.0 (2022-02-08)
------------------
* update dummy gui
* split rqt gui
* Contributors: Shingo Kitagawa

2.0.0 (2021-12-27)
------------------
* update rviz config
* show user ask sentence in rviz
* update debug gui
* use xacro
* update rviz config
* change threshold
* change touch_prx_threshold
* fix typo
* support multi device
* fix typo
* change rviz scene publisher topic name
* update simple ws281x
* change world frame id
* add rmui2 files
* add rmui2 launch
* update rmui launch
* add rmui_dummy
* rename launch files
* use rmui0_link
* use average proximity sensor data
* add shutdown switch script
* flake8
* add mtc2_local
* add rmui1 description
* add rmui1
* add mtc2 in machine
* add env-loader.sh
* change to use touch_prx_threshold
* update simple_ws281x.py
* add rmui_msgs as build_depend
* add quit button for dummy node
* Contributors: Shingo Kitagawa

1.0.3 (2021-08-07)
------------------
* add rmui_local.launch
* add launch_device arg
* update rviz config
* Contributors: Shingo Kitagawa

1.0.2 (2021-07-06)
------------------
* update rviz config
* add moving and waiting signal
* use tabbed layout button
* add rmui_rviz image
* add max_rgb
* change device color depends on prx data
* add imu calibration button
* initialize in init_node
* update ws281x_node
* update led turnoff process
* fix rmui led
* fix conditions
* add led turn_on and off
* update rviz config
* launch timer at the end of init
* update rviz config
* Contributors: Shingo Kitagawa

1.0.1 (2021-06-16)
------------------

1.0.0 (2021-06-06)
------------------
* add rmui_demos
* fix typo in pr2_rmui_dummy.launch
* rename rmui_dummy
* add pr2 launches
* upate namespace
* fix button service type
* update perspectives
* add rqt_contact_sensor_buttons gui
* update rotate_button_layout.yaml
* add all sensor services
* control all sensor contact states in DummyRMUI
* fix import
* refactor simple mpl11a2.py
* flake8
* update simple_mpl115a2.py
* add simple scripts
* add simple_mpl115a2.py
* update rviz config
* add imu calibrated
* do not import
* try matplotlib install
* try except scipy
* add 45 degrees rotate
* do not use robot_localization in dummy mode
* update perspective
* use setbool and default value
* update rviz config
* add rqt_rotate_buttons perspective
* add rotate service for dummy
* add rotate and reset_rotation for dummy
* set quaternion order as  x,y,z,w
* launch rqt_contact_buttons gui for dummy
* add rqt_contact_buttons
* add contact service for dummy rmui
* add contact and release for dummy
* Merge pull request `#8 <https://github.com/knorth55/rmui/issues/8>`_ from knorth55/add-dummy
* flake8
* add dummy_rmui.launch
* change to prx_threshold
* fix typo in dummy_rmui
* fix typo in prx_utils
* check if data is None
* fix average
* add warnings in vcnl4040
* fix typo
* add smbus warnings
* add dummy_rmui_node
* remove unused function
* change import order
* add dummy_rmui
* add prx_utils
* add imu_utils
* add warning for import
* add rmui_client
* change threshold
* Contributors: Shingo Kitagawa

0.0.0 (2020-08-15)
------------------
* update rviz config
* add duration arg
* Merge pull request `#6 <https://github.com/knorth55/rmui/issues/6>`_ from knorth55/rmui-server
* update rviz config
* update markers
* add rmui_server
* fix lint
* add rviz gui
* add rmui rviz config
* add robot_description
* add set_ndof_no_fmc_mode
* add set_radian_unit
* fix typo in read_calib_status
* update imu calibration error
* fix typo in get_imu_calib_msg
* update calib status msg
* publish calib status
* update bno055.py
* print calibration data
* use only rotation
* add read_gravity_acceleration
* update robot_localization.yaml
* fix robot_localization
* update frame link
* add robot_localization
* add covariance
* fix typo
* add n_board param
* update rmui.machine
* add rmui.launch
* add last sensor_board
* add led in rmui
* update ws281x
* set lower brightness for correct color
* add new sensorboard
* add new sensor board
* add correct addresses
* fix typo in rmui_node.py
* fix typo
* fix typo
* add rmui node
* refactor BNO055 nodes
* fix typo
* add vcnl4040_multiplexa.py
* refactor vcnl4040 nodes
* remove unused line
* fix typo
* update vcnl4040_multiplexa_node
* update vcnl4040_node
* try ioerror
* refactor vcnl4040_multiplexa_node.py
* refactor vcnl4040_node.py
* add pca9547_node.py
* update vcnl4040
* add pca9547
* flake8
* pep8
* add linter
* refactor ws281x node
* add brightness
* add ws281x node
* add ws281x python
* update package.xml
* add bno055 node
* add VCNL4040 in rmui_drivers
* initialize publisher before timer
* add python-smbus as run_depend
* use python-smbus
* add vcnl4040 node
* fix typo
* add rmui_drivers package
* Contributors: Shingo Kitagawa
