#!/usr/bin/env python

import roslib
import rospy

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from cola2_msgs.msg import MissionStatus
from cola2_lib.diagnostic_helper import DiagnosticHelper
from cola2_msgs.msg import VehicleStatus
from cola2_msgs.msg import TotalTime
from auv_msgs.msg import NavSts

class VehicleStatusParser:
    """ This node generates the VehicleStatus message from multiple sources."""

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.status = VehicleStatus()

        # Create Publishers
        self.pub_vehicle_sts = rospy.Publisher("/cola2_safety/vehicle_status",
                                               VehicleStatus,
                                               queue_size = 1)
        # Subscriber
        rospy.Subscriber("/diagnostics_agg",
                         DiagnosticArray,
                         self.update_diagnostics,
                         queue_size = 1)

        rospy.Subscriber("/cola2_control/mission_status",
                         MissionStatus,
                         self.updateMissionStatus,
                         queue_size = 1)

        rospy.Subscriber("/cola2_safety/total_time",
                         TotalTime,
                         self.updateTimeout,
                         queue_size = 1)

        rospy.Subscriber("/cola2_navigation/nav_sts",
                         NavSts,
                         self.updateNavSts,
                         queue_size = 1)

    def updateTimeout(self, timeout):
        """Update timeout in vehicle status."""
        self.status.up_time = timeout.total_time

    def updateMissionStatus(self, mission_status):
        """Update mission status information."""
        self.status.current_waypoint = mission_status.current_wp
        self.status.total_waypoints = mission_status.total_wp
        self.status.waypoint_latitude = mission_status.wp_north
        self.status.waypoint_longitude = mission_status.wp_east
        self.status.waypoint_z = mission_status.wp_depth_altitude

    def updateNavSts(self, nav):
        """Update navigation information."""
        self.status.latitude = nav.global_position.latitude
        self.status.longitude = nav.global_position.longitude
        self.status.depth = nav.position.depth
        self.status.heading = nav.orientation.yaw

    def update_diagnostics(self, diagnostics):
        """ Check diagnostics messages to fill vehicle status."""
        for status in diagnostics.status:

            # Get vehicle initialized status
            if __getDiagnostic__(status, '/navigation/ navigator'):
                if __getDiagnostic__(status,
                                     '/navigation/ navigator',
                                     'ekf_init',
                                     'False') == 'True':
                    self.status.vehicle_initialized = True
                else:
                    self.status.vehicle_initialized = False

            # Get battery charge
            if __getDiagnostic__(status, '/safety/ battery'):
                charge = float(__getDiagnostic__(status,
                                                 '/safety/ battery',
                                                 'charge',
                                                 100.0))
                self.status.battery_charge = charge

            # Get IMU data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                imu_age = float(__getDiagnostic__(status,
                                                  '/navigation/ navigator',
                                                  'last_imu_data',
                                                  0.0))
                if imu_age > 10000.0: # To avoid initialization problems
                    imu_age = 0.0
                self.status.imu_data_age = imu_age

            # Get depth data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                depth_age = float(__getDiagnostic__(status,
                                                    '/navigation/ navigator',
                                                    'last_depth_data',
                                                    0.0))
                if depth_age > 10000.0: # To avoid initialization problems
                    depth_age = 0.0
                self.status.depth_data_age = depth_age

            # Get altitude data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                altitude_age = float(__getDiagnostic__(status,
                                                       '/navigation/ navigator',
                                                       'last_altitude_data',
                                                       0.0))
                if altitude_age > 10000.0: # To avoid initialization problems
                    altitude_age = 0.0
                self.status.altitude_data_age = altitude_age

            # Get DVL data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                dvl_age = float(__getDiagnostic__(status,
                                                  '/navigation/ navigator',
                                                  'last_dvl_data',
                                                  0.0))
                if dvl_age > 10000.0: # To avoid initialization problems
                    dvl_age = 0.0
                self.status.dvl_valid_data_age = dvl_age

            # Get GPS data age
            if __getDiagnostic__(status, '/navigation/ navigator'):
                gps_age = float( __getDiagnostic__(status,
                                                   '/navigation/ navigator',
                                                   'last_gps_data',
                                                   0.0))
                if gps_age > 10000.0: # To avoid initialization problems
                    gps_age = 0.0
                self.status.gps_data_age = gps_age

            # Get Navigation data age
            if __getDiagnostic__(status, '/safety/ up_time'):
                nav_age = float(__getDiagnostic__(status,
                                                  '/safety/ up_time',
                                                  'last_nav_data',
                                                  0.0))
                if nav_age > 10000.0: # To avoid initialization problems
                    nav_age = 0.0
                self.status.navigation_data_age = nav_age

            # Get DVL valid data age
            if __getDiagnostic__( status, '/navigation/ teledyne_explorer_dvl'):
                dvl_age = float(__getDiagnostic__(status,
                                                  '/navigation/ teledyne_explorer_dvl',
                                                  'last_good_data',
                                                  0.0))
                if dvl_age > 100000.0:
                    dvl_age = 0.0
                self.status.dvl_valid_data_age = dvl_age

            # Get WIFI age
            if __getDiagnostic__(status, '/control/ teleoperation'):
                wifi_age = float(__getDiagnostic__(status,
                                                   '/control/ teleoperation',
                                                   'last_ack',
                                                   0.0))
                if wifi_age > 100000.0:
                    wifi_age = 0.0
                self.status.wifi_data_age = wifi_age

            # Get modem data age
            if __getDiagnostic__( status, '/safety/ evologics_modem'):
                modem_age = float(__getDiagnostic__(status,
                                                    '/safety/ evologics_modem',
                                                    'last_modem_data',
                                                    0.0))
                if modem_age > 100000.0:
                    modem_age = 0.0
                self.status.modem_data_age = modem_age

            # Get Active controller
            self.status.active_controller = VehicleStatus.CONTROLLER_NONE
            if __getDiagnostic__(status, '/control/ captain'):
                if __getDiagnostic__(status,
                                     '/control/ captain',
                                     'trajectory_enabled',
                                     'False') == 'True':
                    self.status.active_controller = VehicleStatus.CONTROLLER_MISSION

                if __getDiagnostic__(status,
                                     '/control/ captain',
                                     'keep_position_enabled',
                                     'False') == 'True':
                    self.status.active_controller = VehicleStatus.CONTROLLER_KEEP_POSITION

                # TODO: SECTION and GOTO missing, to be added firts in captain!

            # Water inside
            if __getDiagnostic__( status, '/safety/ s2_mon_control_board') or \
               __getDiagnostic__( status, '/control/ s2_actuators'):
                water_main = __getDiagnostic__(status,
                                               '/safety/ s2_mon_control_board',
                                               'water_inside',
                                               'False') == 'True'
                water_fins = __getDiagnostic__(status,
                                               '/control/ s2_actuators',
                                               'water_inside_fins',
                                               'False') == 'True'
                if water_main or water_fins:
                    self.status.water_detected = True
                else:
                    self.status.water_detected = False

        # Publish status message
        self.status.header.stamp = rospy.Time()
        self.pub_vehicle_sts.publish(self.status)

def __getDiagnostic__(status, name, key='none', default=0.0):
    if status.name == name:
        if key != 'none':
            return __getValue__(status.values, key, default)
        else:
            return True
    return False


def __getValue__(values, key, default):
    for pair in values:
        if pair.key == key:
            return pair.value
    return default

if __name__ == '__main__':
    try:
        rospy.init_node('vehicle_status')
        vehicle_status = VehicleStatusParser(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
