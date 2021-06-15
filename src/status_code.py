#!/usr/bin/env python
# Copyright (c) 2019 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

class StatusCode:
    """
    Class to decodify the status code represented in 32bits (4 bytes)
    into different bits representing vehicle status, errors and warnings.
    """

    """
    Definitions of each bit number 
    """
    
    #31-24 TBD

    #23-18 TBD (6 bits left)
    
    RA_EMERGENCY_SURFACE = 17
    RA_ABORT_SURFACE = 16
    RA_ABORT_MISSION = 15
    LOW_BATTERY_WARNING = 14
    BATTERY_ERROR = 13
    WATCHDOG_TIMER = 12
    NAVIGATION_ERROR = 11
    NO_ALTITUDE_ERROR = 10
    WATER_INSIDE = 9
    HIGH_TEMPERATURE = 8
    CURRENT_MISSION_STEPS_BASE = 7 # to 0

    def __init__(self):
        pass

    def unpack(self, status_code):
        """
        Unpacks an status code integer to a dictionary of status, errors and warnings
        """
        status_code_string = format(status_code, '032b')

        ra_emergency_surface = (status_code_string[StatusCode.RA_EMERGENCY_SURFACE] == '1')
        ra_abort_surface = (status_code_string[StatusCode.RA_ABORT_SURFACE] == '1')
        ra_abort_mission = (status_code_string[StatusCode.RA_ABORT_MISSION] == '1')
        low_battery_warning = (status_code_string[StatusCode.LOW_BATTERY_WARNING] == '1')
        battery_error = (status_code_string[StatusCode.BATTERY_ERROR] == '1')
        watchdog_timer = (status_code_string[StatusCode.WATCHDOG_TIMER] == '1')
        navigation_error = (status_code_string[StatusCode.NAVIGATION_ERROR] == '1')
        no_altitude_error = (status_code_string[StatusCode.NO_ALTITUDE_ERROR] == '1')
        water_inside = (status_code_string[StatusCode.WATER_INSIDE] == '1')
        high_temperature = (status_code_string[StatusCode.HIGH_TEMPERATURE] == '1')
        current_mission_steps_base = int(status_code_string[0:StatusCode.CURRENT_MISSION_STEPS_BASE + 1], 2)


        return {
                'ra_emergency_surface': ra_emergency_surface,
                'ra_abort_surface': ra_abort_surface,
                'ra_abort_mission': ra_abort_mission,
                'low_battery_warning': low_battery_warning,
                'battery_error': battery_error,
                'watchdog_timer': watchdog_timer,
                'navigation_error': navigation_error,
                'no_altitude_error': no_altitude_error,
                'water_inside': water_inside,
                'high_temperature': high_temperature,
                'current_mission_steps_base': current_mission_steps_base
                }
