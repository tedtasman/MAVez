# flight_controller.py
# version: 3.1.2
# Original Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2026-03-26
# Organization: PSU UAS

"""
This module is responsible for managing the flight of ardupilot.
"""

# SITL Start Command:
# python3 ./MAVLink/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --console --map --custom-location 38.31527628,-76.54908330,40,282.5

import asyncio
from logging import Logger
from pathlib import Path

from pymavlink import mavutil
from MAVez.enums.armed_status import ArmedStatus
from MAVez.enums.precision_land_mode import PrecisionLandMode
from MAVez.enums.reposition_loiter_mode import RepositionYawMode
from MAVez.mission import Mission
from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage
from MAVez.enums.mav_result import MAVResult
from MAVez.coordinate import Coordinate
import time

from MAVez.mission_item import MissionItem

class FlightController(Controller):
    """
    Manages the flight plan for ardupilot. Extends the Controller class to provide complex flight functionalities.

    Args:
        connection_string (str): The connection string to ardupilot.
        baud (int): The baud rate for the connection. Default is 57600.
        logger (Logger | None): Optional logger for logging flight events.
        message_host (str): The host for messaging.
        message_host (str): The host for the messaging system. Default is "127.0.0.1".
        message_port (int): The port for the messaging system. Default is 5555.
        message_topic (str): The topic prefix for the messaging system. Default is "".
        timesync (bool): Whether to enable time synchronization. Default is False.

    Raises:
        ConnectionError: If the connection to ardupilot fails.

    Returns:
        Flight_Controller: An instance of the Flight_Controller class.
    """

    TIMEOUT_ERROR = 101  # Timeout error code
    BAD_RESPONSE_ERROR = 102  # Bad response error code
    UNKNOWN_MODE_ERROR = 111  # Unknown mode error code
    INVALID_MISSION_ERROR = 301  # Invalid mission error code

    def __init__(self, connection_string: str="tcp:127.0.0.1:5762", 
                 baud: int=57600, 
                 logger: Logger|None=None, 
                 message_host: str="127.0.0.1", 
                 message_port: int=5555, 
                 message_topic: str="",
                 timesync: bool=False,
                 failsafe: bool=True,
    ) -> None:
        # Initialize the controller
        super().__init__(connection_string, logger=logger, baud=baud, message_host=message_host, message_port=message_port, message_topic=message_topic, timesync=timesync)

        self.geofence = Mission(self, type=1)  # type 1 is geofence
        self.__use_failsafe = failsafe

        # initialize mission queue
        self.mission_queue = []

    async def __failsafe(self) -> int:
        if not self.__use_failsafe:
            return 0
        self.logger.warning("[Flight] Failsafe triggered, returning to launch")
        res = await self.set_mode("RTL")

        if res:
            self.logger.critical("[Flight] Error returning to launch in failsafe mode")
            return res

        # wait forever for disarm (completely lock up)
        res = await self.wait_for_disarm(float('inf'))
        if res:
            self.logger.critical("[Flight] Error landing in failsafe mode")
            return res
        return 0

    def decode_error(self, error_code: int) -> str:
        """
        Decode an error code.

        Args:
            error_code (int): The error code to decode.

        Returns:
            str: A string describing the error.
        """

        errors_dict = {
            101: "\nTIMEOUT ERROR (101)\n",
            102: "\nBAD RESPONSE ERROR (102)\n",
            111: "\nUNKNOWN MODE ERROR (111)\n",
            301: "\nINVALID MISSION ERROR (301)\n",
        }

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")

    async def auto_mission_takeoff(self, takeoff_mission_filename: Path) -> int:
        """
        Takeoff ardupilot using an auto mission.

        Args:
            takeoff_mission_filename (str): The file containing the takeoff mission.

        Returns:
            int: 0 if the takeoff was successful, otherwise an error code.
        """

        # Load the takeoff mission from the file
        takeoff_mission = Mission(self)
        response = takeoff_mission.load_mission_from_file(takeoff_mission_filename)

        if response:
            self.logger.critical("[Flight] Takeoff failed, could not load mission")
            return response

        if not takeoff_mission.is_takeoff:
            self.logger.critical("[Flight] Takeoff failed, mission has no takeoff")
            return self.INVALID_MISSION_ERROR

        self.mission_queue.append(takeoff_mission)

        # send the takeoff mission
        response = await takeoff_mission.send_mission()

        # verify that the mission was sent successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, mission not sent")
            return response

        # set the mode to AUTO
        response = await self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, mode not set to AUTO")
            return response
        
        # reset mission index to 0
        response = await self.set_current_mission_index(0, reset=True)
        if response:
            self.logger.critical("[Flight] Takeoff failed, could not reset mission index")
            return response

        # arm ardupilot
        response = await self.arm()

        # verify that ardupilot was armed successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, vehicle not armed")
            return response

        return 0

    def append_mission(self, filename: Path) -> int:
        """
        Append a mission to the mission list.

        Args:
            filename (str): The file containing the mission to append.

        Returns:
            int: 0 if the mission was appended successfully, otherwise an error code.
        """
        # Load the mission from the file
        mission = Mission(self)
        result = mission.load_mission_from_file(filename)

        if result:
            self.logger.critical("[Flight] Could not append mission.")
            return result

        self.logger.info(f"[Flight] Appended mission from {filename} to mission list")
        self.mission_queue.append(mission)
        return 0

    async def wait_for_waypoint(self, target: int) -> int:
        """
        Wait for ardupilot to reach the current waypoint.

        Args:
            target (int): The target waypoint index to wait for.

        Returns:
            int: 0 if the waypoint was reached successfully, otherwise an error code.
        """
        latest_waypoint = -1

        self.logger.debug(f"[Flight] Waiting for waypoint {target} to be reached")

        while latest_waypoint < target:
            response = await self.receive_mission_item_reached()

            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                return response

            latest_waypoint = response

        self.logger.info(f"[Flight] Waypoint {target} reached")
        return 0

    async def auto_send_next_mission(self) -> int:
        """
        Waits for the last waypoint to be reached, clears the mission, sends the next mission, sets mode to auto.

        Returns:
            int: 0 if the next mission was sent successfully, otherwise an error code.
        """
        # Get the current mission
        current_mission = self.mission_queue.pop(0)

        # if the mission list is empty, return
        if len(self.mission_queue) == 0:
            self.logger.info("[Flight] No more missions in list")
            return 0

        # otherwise, set the next mission to the next mission in the list
        else:
            self.logger.info(f"[Flight] Queuing next mission in list of {len(self.mission_queue)} missions")
            next_mission = self.mission_queue[0]

        # calculate the target index
        target_index = len(current_mission) - 1

        # Wait for the target index to be reached
        response = await self.wait_for_waypoint(target_index)

        # verify that the response was received
        if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
            self.logger.critical("[Flight] Failed to wait for next mission.")
            return response

        # Clear the mission
        response = await current_mission.clear_mission()
        if response:
            self.logger.critical("[Flight] Failed to send next mission.")
            return response

        # Send the next mission
        result = await next_mission.send_mission()
        if result:
            self.logger.critical("[Flight] Failed to send next mission.")
            return result

        # set the mode to AUTO
        response = await self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            self.logger.critical("[Flight] Failed to send next mission.")
            return response

        self.logger.info("[Flight] Next mission sent")
        return result

    async def wait_for_landing(self, timeout: float = 60) -> int:
        """
        Wait for ardupilot to signal landed.

        Args:
            timeout (int): The maximum time to wait for the landing status in seconds.

        Returns:
            int: 0 if the landing was successful, otherwise an error code.
        """
        landing_status = -1

        # start receiving landing status
        response = await self.set_message_interval(
            message_type=MAVMessage.EXTENDED_SYS_STATE, interval_us=100000 # 1 second
        )
        if response:
            self.logger.critical("[Flight] Failed waiting for landing.")
            return response

        # wait for landing status to be landed
        start_time = time.time()
        while (
            landing_status != 1
        ):  # 1 for landed, 2 for in air, 3 for taking off, 4 for currently landing, 0 for unknown
            # check for timeout
            if time.time() - start_time > timeout:
                response = self.TIMEOUT_ERROR
                self.logger.error("[Flight] Timed out waiting for landing.")
                return response

            # get the landing status
            response = await self.receive_landing_status()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                self.logger.error("[Flight] Failed waiting for landing.")
                return response

            landing_status = response

        # stop receiving landing status
        response = await self.disable_message_interval(
            message_type=MAVMessage.EXTENDED_SYS_STATE
        )
        if response:
            self.logger.error("[Flight] Error waiting for landing.")
            return response

        return 0

    async def jump_to_next_mission_item(self) -> int:
        """
        Jump to the next mission item.

        Returns:
            int: 0 if the jump was successful, otherwise an error code.
        """

        self.logger.debug("[Flight] Waiting for current mission index")
        # get the latest mission index
        response = await self.receive_current_mission_index(seq=self.__message_seq_by_type["MISSION_CURRENT"])
        if response == self.TIMEOUT_ERROR:
            return response

        # jump to the next mission item
        response = await self.set_current_mission_index(response + 1)
        if response:
            return response

        return 0

    async def wait_for_channel_input(self, channel, value, wait_time=120, value_tolerance=100) -> int:
        """
        Wait for a specified rc channel to reach a given value

        Args:
            channel (int): The channel number to wait for.
            value (int): The value to wait for.
            wait_time (int): The maximum time to wait for the channel to be set in seconds.
            value_tolerance (int): The tolerance range for the set value.

        Returns:
            int: 0 if the channel was set to the desired value, otherwise an error code
        """
        latest_value = -float("inf")
        start_time = time.time()

        # set the channel to be received
        channel = f"chan{channel}_raw"

        self.logger.debug(f"[Flight] Waiting for channel {channel} to be set to {value}")

        # only wait for the channel to be set for a certain amount of time
        while time.time() - start_time < wait_time:
            # get channel inputs
            response = await self.receive_channel_input()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                self.logger.critical("[Flight] Failed waiting for channel input.")
                return response

            # channel key is 'chanX_raw' where X is the channel number
            latest_value = getattr(response, channel)

            # check if the value is within the tolerance range
            if (
                latest_value > value - value_tolerance
                and latest_value < value + value_tolerance
            ):
                self.logger.info(f"[Flight] Channel {channel} set to {latest_value}")

                return 0

        self.logger.critical(
            f"[Flight] Timed out waiting for channel {channel} to be set to {value}"
        )
        return self.TIMEOUT_ERROR
    
    async def set_geofence(self, geofence_filename: Path) -> int:
        """
        Send and enable the geofence from a file.

        Args:
            geofence_filename (str): The file containing the geofence mission.

        Returns:
            int: 0 if the geofence was set successfully, otherwise an error code.
        """
        # Load the geofence mission from the file
        response = self.geofence.load_mission_from_file(geofence_filename)

        if response:
            self.logger.critical("[Flight] Geofence failed, could not load mission")
            return response

        if not self.geofence.is_geofence:
            self.logger.critical("[Flight] Geofence failed, mission is not a geofence")
            return self.INVALID_MISSION_ERROR

        # send the geofence mission
        response = await self.geofence.send_mission()

        # verify that the mission was sent successfully
        if response:
            self.logger.critical("[Flight] Geofence failed, mission not sent")
            return response

        self.logger.debug("[Flight] Geofence sent")

        response = await self.enable_geofence()
        if response:
            self.logger.critical("[Flight] Geofence failed, could not be enabled")
            return response
        
        self.logger.debug("[Flight] Geofence set and enabled")
        return 0
    
    async def wait_for_position_reached(self, position: Coordinate, tolerance_m: float, timeout_s: float) -> int:
        """Wait until a specified position is reached

        Args:
            position (Coordinate): Position to wait for
            tolerance_m (float): Tolerance for accepting position in meters
            timeout_s (float): Maximum time to wait

        Returns:
            int: 0 on reaching position else error code
        """
        self.logger.info(f"[Flight] Waiting up to {timeout_s} seconds to reach {position}")
        # poll global_position_int
        timeout_time = time.time() + timeout_s
        while (time.time() < timeout_time):  

            # get position
            response = await self.receive_gps()

            if not isinstance(response, Coordinate):
                self.logger.error("[Flight] Failed waiting for position reached.")
                return response
            
            elif position.distance_to(response, include_alt=True) < tolerance_m:
                return 0

        response = self.TIMEOUT_ERROR
        self.logger.error("[Flight] Timed out waiting for position reached.")
        return response

    
    async def wait_for_altitude_reached(self, altitude_m: float, tolerance_m: float, timeout_s: float) -> int:
        """Wait until a specified altitude is reached

        Args:
            altitude_m (float): Altitude to wait for in meters  
            tolerance_m (float): Tolerance for accepting altitude in meters
            timeout_s (float): Maximum time to wait

        Returns:
            int: 0 on reaching altitude else error code
        """
        self.logger.info(f"[Flight] Waiting up to {timeout_s} seconds to reach {altitude_m}m")
        # poll global_position_int
        timeout_time = time.time() + timeout_s
        while (time.time() < timeout_time):  

            # get position
            response = await self.receive_gps()

            if not isinstance(response, Coordinate):
                self.logger.error("[Flight] Failed waiting for altitude reached.")
                return response
            
            elif abs(altitude_m - response.altitude_m) < tolerance_m:
                return 0

        response = self.TIMEOUT_ERROR
        self.logger.error("[Flight] Timed out waiting for altitude reached.")
        return response


    async def takeoff(
            self, 
            altitude_m: float, 
            timeout_s: float, 
            tolerance_m: float = 2, 
            pitch_deg: float = 0, 
            require_horizontal_position: bool = True,
            critical = False
    ) -> int: 
        """Takeoff at current position and wait for altitude reached

        Args:
            altitude_m (float): Altitude to ascend to
            timeout_s (float): Maximum wait time to reach altitude
            tolerance_m (float, optional): Tolerance for altitude acceptance in meters. Defaults to 2.
            pitch_deg (float, optional): Pitch to ascend at (planes only). Minimum pitch if airspeed sensor present, else desired pitch.
            require_horizontal_position (bool, optional): Require autopilot to have control over its horizontal position. Defaults to True.
            critical (bool, optional): Flag for triggering failsafe on error. Defaults to False.

        Returns:
            int: 0 on success else error code
        """
        res = await self.send_takeoff(altitude_m, pitch_deg, require_horizontal_position)
        if res:
            self.logger.critical("[Flight] Takeoff failed")
            await self.__failsafe() if critical else None
            return res
        
        res = await self.wait_for_altitude_reached(altitude_m, tolerance_m, timeout_s)
        if res:
            self.logger.critical("[Flight] Takeoff failed")
            await self.__failsafe() if critical else None
            return res
        
        self.logger.info("[Flight] Takeoff complete")
        return 0
    

    async def wait_for_disarm(self, timeout_s: float) -> int:

        self.logger.info(f"[Flight] Waiting up to {timeout_s} seconds for disarmed")
        # poll global_position_int
        timeout_time = time.time() + timeout_s
        while (time.time() < timeout_time):  
            response = await self.is_armed()
            if response == ArmedStatus.NOT_ARMED:
                return 0
            await asyncio.sleep(0.5)

        response = self.TIMEOUT_ERROR
        self.logger.error("[Flight] Timed out waiting for disarm.")
        return response


    async def go_to(
            self,
            position: Coordinate, 
            timeout_s: float,
            accept_radius_m: float,
            loiter_radius_m: float = 0, 
            speed_mps: float = -1,
            loiter_mode: RepositionYawMode = RepositionYawMode.USE_YAW,
            change_mode: bool = False,
            relative_yaw: bool = False,
    ) -> int:
        """Reposition and wait for arrival

        Args:
            position (Coordinate): Target location
            timeout_s (float): Maximum wait time for arrival
            accept_radius_m (float): Tolerance for arrival proximity in meters
            loiter_radius_m (float, optional): Radius for planes to loiter. Defaults to 0.
            speed_mps (float, optional): Speed to travel at in meters/second. If -1 or not specified, uses system default.
            loiter_mode (RepositionYawMode, optional): Loitering behavior option. Defaults to RepositionYawMode.USE_YAW.
            change_mode (bool, optional): Flag to change mode to guided. Defaults to False.
            relative_yaw (bool, optional): Flag to yaw relative to vehicle heading instead of North. Defaults to True.

        Returns:
            int: 0 for success, o.w. error code.
        """
        
        res = await self.send_reposition(
            position,
            loiter_radius_m,
            speed_mps,
            loiter_mode,
            change_mode,
            relative_yaw,
        )

        if res:
            self.logger.error(f"[Flight] Failed to go to position, {res}")
            return res
        
        res = await self.wait_for_position_reached(position, accept_radius_m, timeout_s)
        if res:
            self.logger.error(f"[Flight] Failed to go to position, {res}")
            return res

        return 0
        

    async def land(
            self, 
            position: Coordinate,
            abort_altitude_m: float,
            timeout_s: float,
            precision_land_mode: PrecisionLandMode = PrecisionLandMode.DISABLED,
            critical = True
    ) -> int: 
        """Land at a given position

        Args:
            position (Coordinate): Position to land at
            abort_altitude_m (float): Minimum altitude to trigger abort
            timeout_s (float): Maximum time to wait for successful landing
            precision_land_mode (PrecisionLandMode, optional): Flag for using precision land mode. Defaults to PrecisionLandMode.DISABLED.
            critical (bool, optional): Flag for triggering failsafe on error. Defaults to False.

        Returns:
            int: 0 for success, o.w. error code
        """
        
        res = await self.set_mode("AUTO")
        position.altitude_m = 0
        mission = Mission(self)

        waypoint = MissionItem(
            seq=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            auto_continue=1,
            coordinate=position,
            param1=abort_altitude_m,
            param2=precision_land_mode.value,
            param3=0,
            param4=position.heading_deg,
            type=0
        )
        landing = MissionItem(
            seq=1,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            current=0,
            auto_continue=1,
            coordinate=position,
            param1=abort_altitude_m,
            param2=precision_land_mode.value,
            param3=0,
            param4=position.heading_deg,
            type=0
        )
        waypoint2 = MissionItem(
            seq=2,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            auto_continue=1,
            coordinate=position,
            param1=abort_altitude_m,
            param2=precision_land_mode.value,
            param3=0,
            param4=position.heading_deg,
            type=0
        )

        mission.add_mission_item(waypoint)
        mission.add_mission_item(landing)
        mission.add_mission_item(waypoint2)
        res = await mission.send_mission()

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Landing command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for landing")
            return self.BAD_RESPONSE_ERROR
        elif res != 0:
            self.logger.error(f"[Controller] Failed to land: {MAVResult.string(res)}")
            return res
        
        res = await self.wait_for_landing(timeout_s)
        if res:
            self.logger.critical("[Flight] Landing failed")
            await self.__failsafe() if critical else None
            return res
        
        await self.disarm()
        
        # send a dummy mission to escape landing sequence
        dummy_mission = Mission(self)
        dummy_mission.add_mission_item(
            MissionItem(
                seq=0,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0,
                auto_continue=1,
                coordinate=position,
                param1=0,
                param2=0,
                param3=0,
                param4=position.heading_deg,
                type=0
            )
        )
        dummy_mission.add_mission_item(
            MissionItem(
                seq=1,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0,
                auto_continue=1,
                coordinate=position,
                param1=0,
                param2=0,
                param3=0,
                param4=position.heading_deg,
                type=0
            )
        )

        res = await dummy_mission.send_mission()

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Landing command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for landing")
            return self.BAD_RESPONSE_ERROR
        elif res != 0:
            self.logger.error(f"[Controller] Failed to land: {MAVResult.string(res)}")
            return res
        
        res = await self.wait_for_landing(timeout_s)
        if res:
            self.logger.critical("[Flight] Landing failed")
            await self.__failsafe() if critical else None
            return res
        
        self.logger.info("[Flight] Landing complete")
        return 0
    
    async def land_here(
            self,
            abort_altitude_m: float,
            timeout_s: float,
            precision_land_mode: PrecisionLandMode = PrecisionLandMode.DISABLED,
            critical = False
    ) -> int:
        """Land at current position

        Args:
            abort_altitude_m (float): Minimum altitude to trigger abort
            timeout_s (float): Maximum time to wait for successful landing
            precision_land_mode (PrecisionLandMode, optional): Flag for using precision land mode. Defaults to PrecisionLandMode.DISABLED.
            critical (bool, optional): Flag for triggering failsafe on error. Defaults to False.

        Returns:
            int: 0 for success, o.w. error code
        """
        position = await self.receive_gps()
        if not isinstance(position, Coordinate):
            self.logger.error(f"[Flight] Failed to get current position for landing, {position}")
            return position

        res = await self.land(
            position,
            abort_altitude_m,
            timeout_s,
            precision_land_mode,
            critical
        )

        return res
    
    async def is_armed(self) -> int:
        """Check if the vehicle is armed. Will attempt to run prearm checks to test

        Returns:
            int: 0 for unarmed, 1 for armed, else error code
        """
        # prearm checks returns 0 if accepted, which means unarmed
        # prearm checks returns 1 if temporarily rejected, which means armed
        return await self.run_prearm_checks()

