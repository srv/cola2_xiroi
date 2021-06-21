#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
@@>This node simulates the batteries (e.g. decaying battery charge) and different diagnostics from internal sensors (water sensors, temperature, etc).<@@
"""

# ROS imports
import rospy

from sensor_msgs.msg import BatteryState, Temperature, FluidPressure, RelativeHumidity
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from cola2_lib.rosutils import param_loader
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Empty, Bool
from cola2_xiroi.msg import PowerReading
import numpy as np

#TODO: This node should be generic. It means that it should read from a yaml file how many temperature, water ...
#      ... and humidity sensors have to simulate (i.e., generate diagnostics) and with which names.

class SimSensorsGirona500:
    def __init__(self, name):
        self.name = name
        self.battery_level = 93
        self.battery_consumption_per_second = 0.004
        self.update_battery_time = 3
        self.get_config()
        self.first_time =True
        right_current_buffer = np.array([])
        self.sensor_buffer_size = 20
        buffer = np.array([])

        # Set up diagnostics
        self.diagnostic_bat = DiagnosticHelper("/batteries", "Simulated")
        self.diagnostic_bat_cylinder = DiagnosticHelper("/batteries_cylinder", "Simulated")
        self.diagnostic_pc_cylinder = DiagnosticHelper("/pc_cylinder", "Simulated")

        # Create publisher
        namespace = rospy.get_namespace()
        self.pub_battery = rospy.Publisher(
            namespace + 'batteries/status', BatteryState, queue_size=2)
        self.pub_bat_humidity = rospy.Publisher(
            namespace + 'batteries_cylinder/humidity', RelativeHumidity, queue_size=2)
        self.pub_bat_pressure = rospy.Publisher(
            namespace + 'batteries_cylinder/pressure', FluidPressure, queue_size=2)
        self.pub_bat_temperature = rospy.Publisher(
            namespace + 'batteries_cylinder/temperature', Temperature, queue_size=2)
        self.pub_bat_water_detected = rospy.Publisher(
            namespace + 'batteries_cylinder/water_detected', Bool, queue_size=2)

        self.pub_pc_humidity = rospy.Publisher(
            namespace + 'pc_cylinder/humidity', RelativeHumidity, queue_size=2)
        self.pub_pc_pressure = rospy.Publisher(
            namespace + 'pc_cylinder/pressure', FluidPressure, queue_size=2)
        self.pub_pc_temperature = rospy.Publisher(
            namespace + 'pc_cylinder/temperature', Temperature, queue_size=2)
        self.pub_pc_water_detected = rospy.Publisher(
            namespace + 'pc_cylinder/water_detected', Bool, queue_size=2)

        # Init periodic check timer
        rospy.Timer(rospy.Duration(1.0), self.pub_pc_cylinder_sensors)
        rospy.Timer(rospy.Duration(1.0), self.pub_batteries_cylinder_sensors)
        rospy.Timer(rospy.Duration(self.update_battery_time), self.pub_batteries_sensors)


    def pub_batteries_sensors(self, event):

        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = 12.3
        battery_msg.current = 5.2
        battery_msg.current = 10.0
        battery_msg.capacity = 50.0
        battery_msg.charge = 61.5
        battery_msg.design_capacity = 50.0
        battery_msg.percentage = 61.5
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.present = True
        self.pub_battery.publish(battery_msg)

        # Reduce battery level
        self.battery_level = (self.battery_level -
                              self.battery_consumption_per_second * self.update_battery_time)
        if self.battery_level < 0.0:
            self.battery_level = 0.0

        # Publish diagnostic message
        self.diagnostic_bat.add('charge', str(battery_msg.charge))
        self.diagnostic_bat.add('voltage', str(battery_msg.voltage))
        self.diagnostic_bat.add('minutes', str(self.battery_level * 3.15))
        self.diagnostic_bat.add('status', "DISCHARGING")
        if battery_msg.charge > 15.0:
            self.diagnostic_bat.set_level(DiagnosticStatus.OK)
        elif battery_msg.charge > 5.0:
            self.diagnostic_bat.set_level(DiagnosticStatus.WARN, 'Low Battery')
        else:
            self.diagnostic_bat.set_level(DiagnosticStatus.ERROR, 'Recharge battery!')

    def get_config(self):
        param_dict = {'battery_level': ('initial_battery_level', 100),
                      'battery_consumption_per_second': ('battery_consumption_per_second', 0)}

        param_loader.get_ros_params(self, param_dict)

    def update_heart_beat(self, hb):
        # Todo if no heart beat is received in X seconds, abort mission
        pass

    def pub_pc_cylinder_sensors(self, event):
        """PC Cylinder humidity"""
        hum_msg = RelativeHumidity()
        hum_msg.header.stamp = rospy.Time.now()
        hum_msg.relative_humidity = 45
        hum_msg.variance = 0
        self.pub_pc_humidity.publish(hum_msg)
        """PC Cylinder temperature"""
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = 57.4
        temp_msg.variance = 0
        self.pub_pc_temperature.publish(temp_msg)
        """PC Cylinder pressure """
        press_msg = FluidPressure()
        press_msg.header.stamp = rospy.Time.now()
        press_msg.fluid_pressure = 101300
        press_msg.variance = 0
        self.pub_pc_pressure.publish(press_msg)
        """PC Cylinder Water detected"""
        water_detected_msg = Bool()
        water_detected_msg.data=False
        self.pub_pc_water_detected.publish(water_detected_msg)

        self.diagnostic_pc_cylinder.add("temperature", str(temp_msg.temperature))
        self.diagnostic_pc_cylinder.add("pressure", str(press_msg.fluid_pressure))
        self.diagnostic_pc_cylinder.add("water", str(water_detected_msg.data))
        self.diagnostic_pc_cylinder.set_level(DiagnosticStatus.OK)

    def pub_batteries_cylinder_sensors(self, event):
        """Batteries Cylinder humidity"""
        hum_msg = RelativeHumidity()
        hum_msg.header.stamp = rospy.Time.now()
        hum_msg.relative_humidity = 38
        hum_msg.variance = 0
        self.pub_bat_humidity.publish(hum_msg)
        """Batteries Cylinder temperature"""
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = 31.2
        temp_msg.variance = 0
        self.pub_bat_temperature.publish(temp_msg)
        """Batteries Cylinder pressure """
        press_msg = FluidPressure()
        press_msg.header.stamp = rospy.Time.now()
        press_msg.fluid_pressure = 101325
        press_msg.variance = 0
        self.pub_bat_pressure.publish(press_msg)
        """Batteries Cylinder Water detected"""
        water_detected_msg = Bool()
        water_detected_msg.data = False
        self.pub_bat_water_detected.publish(water_detected_msg)

        self.diagnostic_bat_cylinder.add("temperature", str(temp_msg.temperature))
        self.diagnostic_bat_cylinder.add("pressure", str(press_msg.fluid_pressure))
        self.diagnostic_bat_cylinder.add("water", str(water_detected_msg.data))
        self.diagnostic_bat_cylinder.set_level(DiagnosticStatus.OK)

    # def pub_batteries_sensors(self, event):

    #     battery_msg = BatteryState()
    #     battery_msg.header.stamp = rospy.Time.now()
    #     battery_msg.voltage = 30.6
    #     battery_msg.current = -30.02
    #     battery_msg.charge = self.battery_level
    #     battery_msg.capacity = float('nan')
    #     battery_msg.design_capacity = float('nan')
    #     battery_msg.percentage = float('nan')
    #     battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    #     battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    #     battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    #     battery_msg.present = True
    #     self.pub_battery.publish(battery_msg)

    #     # Reduce battery level
    #     self.battery_level = (self.battery_level -
    #                           self.battery_consumption_per_second * self.update_battery_time)
    #     if self.battery_level < 0.0:
    #         self.battery_level = 0.0

    #     # Publish diagnostic message
    #     self.diagnostic_bat.add('charge', str(battery_msg.charge))
    #     self.diagnostic_bat.add('voltage', str(battery_msg.voltage))
    #     self.diagnostic_bat.add('minutes', str(self.battery_level * 3.15))
    #     self.diagnostic_bat.add('status', "DISCHARGING")
    #     if battery_msg.charge > 15.0:
    #         self.diagnostic_bat.set_level(DiagnosticStatus.OK)
    #     elif battery_msg.charge > 5.0:
    #         self.diagnostic_bat.set_level(DiagnosticStatus.WARN, 'Low Battery')
    #     else:
    #         self.diagnostic_bat.set_level(DiagnosticStatus.ERROR, 'Recharge battery!')


if __name__ == '__main__':
    try:
        rospy.init_node('sim_internal_sensors')
        sim_sensors_girona500 = SimSensorsGirona500(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
