^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cola2_s2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming (2016/11/28)
------------------------
* Check max depth sensor offset to be applied (2m).
* Add depth sensor offset value to be used when automatic offset is not used.
* Modified initial enable/disable thrusters behaviour.
* Navigator: now takes into account orientation in addLandmark srv.
* Add TF for seaking_profiler & corrected TF fot echosounder15.
* Add service to call resetLandmarks method.
* Replaced depth sensor from 30 to 100 bars.
* Modified basic sparus RVIZ configuration.
* If depth sensor fails returns a -99. Check this condition in navigation filter.
* Add modifications in sim sparus launch to enable simulated profiler.
* If some sensor is not initialized set navigator diagnostic to warning.
* Navigator should not generate errors if open out of water.
* Add node_bag.py at core launch files.
* Add simulated depth in sparus sim_nav_sensors.
* Corrected DVL tf.
* Add combination to enable/disable thrusters from logitech FX10.
* Renamed keep position s2 to 3dof.
* Improved launch files, delete old configs, add new launchs/folders/...
* Moving pressure sensor parameters to the configuration file.
* Automatisation of the magnetometer calibration process.
* Add reset_navigator_action service.
* Modifications on mission file to alllow to define velocities and tolerance.
* Navigator: adapted to RangeDetection message changes.
* Add publish marker for range updates.
* Add updateRange and addLandmark functionalities.
* Simulation now has Sparus II dynamic parameters.
* Navigator: added landmark debug messages.
* Rviz flag included in the launch file.
* Navigator: added usbl publish.
* Add rule to avoid control Z when both current Z and requested Z close to 0.0.
* Add Simulated DVL.
* Modifications in pose control from joystick.
* Update .rosinstall file.
* Replace PoseWithCovarianceStamped message for landmark update by cola2_msgs/Detection message.
* Adapted to changes in cola2_lib.
* Remove references to MORPH.
* Replaced pressure sensor offset with service /cola2_navigation/set_depth_sensor_offset.
* param_logger added to save all params in the bagfiles.
* Launch: now first launch to include is s2_configs.launch.
* Navigator: gps_ned published pointing up.
* Add logitech_fx10_to_teleoperation_s2 specific for sparus2.
* Profiler included in the S2 payload.
* Add 3DS model for sparus in RVIz.
* Update analyzers.
* Add min DVL good data value. 
* Simulated GPS always publish data but quality depends on vehicle depth.
* Add example mission file.
* Change env.bash path from cola2 to cole_core.
* Rosintall for sparus-II.
* Messages are now defined in cola2_msgs.
* Rename config files (remove _s2) and correct launches.
* Launch and config files for SPARUS-II added.
* Modified controller s2 cfg file package.
* Add necessary files for the s2_controller.
* Add config files, launchs and data files for SII.
* Replace EMUSBMS by EmusBms.
* Initial commit.
* Contributors: Narcis, Narcis Palomeras, juandhv, sparus.
