# mav_controller.py
# version: 3.8.0
# Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2026-03-26
# Organization: PSU UAS

"""
This module is responsible for controlling ardupilot.
"""

import asyncio
from collections import defaultdict
import time
from logging import Logger
from typing import Callable
from pymavlink import mavutil
from pyparsing import Any # type: ignore[import]

from lingo import Publisher, Message

from MAVez.translate_message import translate_message
from MAVez.coordinate import Coordinate
from MAVez.safe_logger import SafeLogger

from MAVez.enums.mav_landed_state import MAVLandedState
from MAVez.enums.mav_mission_result import MAVMissionResult
from MAVez.enums.mav_result import MAVResult
from MAVez.enums.mav_message import MAVMessage
from MAVez.enums.reposition_loiter_mode import RepositionYawMode


class Controller:
    """
    Controller class for atomic MAVLink communication with ardupilot.

    Args:
        connection_string (str): The connection string for ardupilot. Default is "tcp:127.0.0.1:5762" used for SITL.
        baud (int): The baud rate for the connection. Default is 57600.
        logger: Logger instance for logging messages (optional).
        message_host (str): The host for the messaging system. Default is "127.0.0.1".
        message_port (int): The port for the messaging system. Default is 5555.
        message_topic (str): The topic prefix for the messaging system. Default is "".

    Raises:
        ConnectionError: If the connection to ardupilot fails.
    """

    # error codes
    TIMEOUT_ERROR = 101
    BAD_RESPONSE_ERROR = 102
    UNKNOWN_MODE = 111

    TIMEOUT_DURATION = 5  # timeout duration in seconds

    def __init__(self, connection_string: str = "tcp:127.0.0.1:5762", 
                 baud: int = 57600, 
                 logger: Logger | None = None, 
                 message_host: str = "127.0.0.1", 
                 message_port: int = 5555, 
                 message_topic: str = "",
                 timesync: bool = False) -> None:
        """
        Initialize the controller.

        Args:
            connection_string (str): The connection string for ardupilot.
            baud (int): The baud rate for the connection.
            logger: Logger instance for logging messages (optional).
            message_host (str): The host for the messaging system. Default is "127.0.0.1".
            message_port (int): The port for the messaging system. Default is 5555.
            message_topic (str): The topic prefix for the messaging system. Default is "".
            timesync (bool): Whether to enable time synchronization. Default is False.
        Raises:
            ConnectionError: If the connection to ardupilot fails.

        Returns:
            None
        """

        self.logger = SafeLogger(logger)

        self.master: mavutil.mavfile = mavutil.mavlink_connection(connection_string, baud=baud)  # type: ignore

        response = self.master.wait_heartbeat(
            blocking=True, timeout=self.TIMEOUT_DURATION
        ) 
        # check if the connection was successful
        if not response:
            self.logger.error("[Controller] Connection failed")
            raise ConnectionError("Connection failed")
        self.logger.info(f"[Controller] Connection successful. Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")  # type: ignore

        self.msg_queue = asyncio.Queue()
        self.pub = Publisher(host=message_host, port=message_port, outbound_queue=self.msg_queue)
        self.message_topic = message_topic
        self.logger.info(f"[Controller] Publisher initialized at {message_host}:{message_port}")
        
        # message variables
        self.__running = asyncio.Event()
        self.__message_pump_task = None
        self.__clock_sync_task = None
        self.__waiters_by_type: defaultdict[str, list[asyncio.Event]] = defaultdict(list)
        self.__message_seq_by_type: defaultdict[str, int] = defaultdict(int)
        self.__latest_messages: dict[str, Message] = {}

        # clock sync variables
        self.timesync = timesync
        self.rtt_ns = None
        self.start_time = time.monotonic_ns()
        self.offset_ns = None
        self.last_sync_time = 0
        self.ROLLING_WINDOW = 50
        self.CLOCK_SYNC_INTERVAL = 3  # seconds
        self.local_samples = []
        self.peer_samples = []

    def decode_error(self, error_code: int) -> str:
        """
        Decode the error code into a human-readable string.

        Args:
            error_code (int): The error code to decode.

        Returns:
            str: A human-readable error message.
        """
        errors_dict = {
            101: "\nRESPONSE TIMEOUT ERROR (101)\n",
            102: "\nBAD RESPONSE ERROR (102)\n",
            111: "\nUNKNOWN MODE ERROR (111)\n",
        }

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")
    
    async def start(self):
        """
        Start the controller by initiating the message pump.

        Returns:
            None
        """
        if self.__message_pump_task is None:
            self.__running.set()
            self.__message_pump_task = asyncio.create_task(self.message_pump())
            self.logger.debug("[Controller] Message pump started")
        
        if self.timesync:
            await self.sync_clocks()
            if self.__clock_sync_task is None:
                self.__clock_sync_task = asyncio.create_task(self.clock_synchronizer())
                self.logger.debug("[Controller] Clock synchronizer started")

    async def stop(self):
        """
        Stop the controller by cancelling running tasks.

        Returns:
            None
        """
        self.logger.info("[Controller] Shutting down...")

        self.__running.clear()
        if self.__message_pump_task:
            self.__message_pump_task.cancel()
            try:
                await self.__message_pump_task
            except asyncio.CancelledError:
                self.logger.debug("[Controller] Message pump stopped")
            self.__message_pump_task = None
        
        if self.__clock_sync_task:
            self.__clock_sync_task.cancel()
            try:
                await self.__clock_sync_task
            except asyncio.CancelledError:
                self.logger.debug("[Controller] Clock synchronizer stopped")
            self.__clock_sync_task = None

        if self.pub:
            self.pub.close()
            self.logger.debug("[Controller] Publisher closed")

        self.logger.info("[Controller] Shutdown complete")
    
    async def message_pump(self):
        """
        Continuously read MAVLink messages and push them into a queue.

        Returns:
            None
        """
        loop = asyncio.get_running_loop()
        if self.pub:
            self.pub.start()
        try:
            while self.__running.is_set():
                try:
                    # use run_in_executor to make recv_match async
                    mav_msg = await loop.run_in_executor(None, lambda: self.master.recv_match(blocking=True))

                    if mav_msg:
                        msg = translate_message(mav_msg, self.message_topic)
                        if msg:
                            # update cache and seq with new message
                            self.__latest_messages[mav_msg.get_type()] = msg
                            self.__message_seq_by_type[mav_msg.get_type()] += 1
                            # wake up all waiters
                            for waiter in self.__waiters_by_type[mav_msg.get_type()]:
                                waiter.set()
                            # publish message for listeners
                            await self.msg_queue.put(msg)

                except Exception as e:
                    self.logger.error(f"[Controller] Error in message pump: {e}")
        
        # Handle shutdown gracefully
        except asyncio.CancelledError:
            self.logger.debug("[Controller] Message pump cancelled")
        finally:
            self.logger.debug("[Controller] Message pump stopped")
            if self.pub:
                self.pub.close()

    async def __aenter__(self):
        await self.start()
        return self
    
    async def __aexit__(self, exc_type, exc_value, traceback):
        await self.stop()
    
    async def request_message(self, message_type: MAVMessage, qualifier: Callable[[dict], bool] = lambda _: True, timeout: float = 5.0) -> dict | None:
        request = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
            0,  # confirmation
            1,  # param1
            message_type.value,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        message_seq = self.get_message_seq(message_type.name) + 1
        self.send_message(request)
        return await self.receive_message(message_type, message_seq, qualifier, timeout)

    async def receive_message(self, message_type: MAVMessage, seq: int = -1, qualifier: Callable[[dict], bool] = lambda _: True, timeout: float = 5.0) -> dict | None:
        """
        Wait for a specific MAVLink message type from ardupilot.

        Args:
            message_type (str): The type of MAVLink message to wait for.
            seq (int, optional): The minimum sequence of the message type desired. If omitted, waits for the next new message
            qualifier (Callable[[dict], bool]): Additional requirement for message to be returned. By default the first matching message is returned.
            timeout (float): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            dict | None: Dictionary representation of MAVLink message if successful, None if the response timed out.
        """
        # Set seq to next new message by default
        if seq == -1:
            seq = self.__message_seq_by_type[message_type.name] + 1

        # Check latest messages cache if seq already met
        latest = self.__latest_messages.get(message_type.name)
        if self.__message_seq_by_type[message_type.name] >= seq and latest is not None and qualifier(latest.header):
            return latest.header

        # Add signal to wake up when new message received
        signal = asyncio.Event()
        self.__waiters_by_type[message_type.name].append(signal)

        try:
            start_time = time.monotonic()
            while time.monotonic() - start_time < timeout:
                # wait up to timeout for new message signal
                remaining = timeout - (time.monotonic() - start_time)
                try:
                    await asyncio.wait_for(signal.wait(), timeout=remaining)
                except asyncio.TimeoutError:
                    return
                # verify that the message is new and valid and meets qualifications
                msg = self.__latest_messages.get(message_type.name)
                if msg is not None and qualifier(msg.header) and self.__message_seq_by_type[message_type.name] >= seq:
                    return msg.header
                # reset signal on unwanted message
                signal.clear()
            
            return
        
        # clean up signal from waiters
        finally:
            waiters = self.__waiters_by_type.get(message_type.name)
            if waiters is not None:
                try:
                    waiters.remove(signal)
                except ValueError:
                    pass
                if not waiters:
                    del self.__waiters_by_type[message_type.name]


    async def receive_mission_request(self, seq: int, timeout: float = 5.0) -> int:
        """
        Wait for a mission request from ardupilot.

        Args:
            timeout (float): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: Mission index if a mission request was received, 101 if the response timed out, 102 if a bad response was received.
        """
        message = await self.receive_message(MAVMessage.MISSION_REQUEST, seq, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive mission request timed out")
            return self.TIMEOUT_ERROR
        elif message.get('seq') is not None:
            self.logger.debug(f"[Controller] Received mission request for index: {message['seq']}")
            return message['seq'] if message['seq'] is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for mission request")
            return self.BAD_RESPONSE_ERROR
        
    async def receive_mission_ack(self, seq: int, timeout: float = 5.0) -> int:
        """
        Wait for a mission ack from ardupilot.

        Args:
            timeout (float): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: 0 if a mission ack was received, error code if there was an error, 101 if the response timed out.
        """
        message = await self.receive_message(MAVMessage.MISSION_ACK, seq, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive mission ack timed out")
            return self.TIMEOUT_ERROR
        elif message.get('type') is not None:
            if message['type'] == 0:  # MAV_MISSION_ACCEPTED
                self.logger.debug("[Controller] Received mission ack: MAV_MISSION_ACCEPTED")
                return 0
            else:
                self.logger.error(f"[Controller] Received mission ack with error: {MAVMissionResult.string(result_code=message['type'])}")
                return message['type'] if message['type'] is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for mission ack")
            return self.BAD_RESPONSE_ERROR

    def send_message(self, message):
        """
        Send a MAVLink message to ardupilot.

        Args:
            message: The MAVLink message to send.

        Returns:
            None
        """
        self.master.mav.send(message) # type: ignore

    async def send_command_with_ack(self, message, command_id: int, timeout: int) -> int:
        """Send a MAVLink command message and wait for the corresponding COMMAND_ACK

        Args:
            message (MAVLink message): MAVLink message to be sent
            command_id (int): Integer ID representation of the MAVLink command 
            timeout (int): Time to wait for ack

        Returns:
            int: COMMAND_ACK result if received, TIMEOUT_ERROR if timeout, or BAD_RESPONSE_ERROR if non-COMMAND_ACK received
        """

        if not self.__running.is_set():
            self.logger.warning("[Controller] Cannot receive command ack if controller is not running. Call `controller.start` first")

        next_seq = self.__message_seq_by_type["COMMAND_ACK"] + 1
        self.send_message(message)

        message = await self.receive_message(
            message_type=MAVMessage.COMMAND_ACK, 
            seq=next_seq, 
            qualifier=lambda msg: msg.get("command") == command_id,
            timeout=timeout
        )

        if message is None:
            return self.TIMEOUT_ERROR
        elif message.get('result') is not None:
            return message['result']
        else:
            return self.BAD_RESPONSE_ERROR

    def send_mission_count(self, count, mission_type=0) -> int:
        """
        Send the mission count to ardupilot.

        Args:
            count (int): The number of mission items.
            mission_type (int): The type of mission (default is 0 for MISSION_TYPE 0).

        Returns:
            int: 0 if the mission count was sent successfully.
        """
        self.master.mav.mission_count_send( # type: ignore
            0,  # target_system
            0,  # target_component
            count,  # count
            mission_type,  # mission_type
        )
        self.logger.debug(f"[Controller] Sent mission count: {count}")
        return 0

    async def receive_mission_item_reached(self, seq: int=-1, timeout: int = 240) -> int:
        """
        Wait for a mission item reached message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 240 seconds.

        Returns:
            int: The sequence number of the reached mission item if received, TIMEOUT_ERROR (101) if the response timed out.
        """
        message = await self.receive_message(MAVMessage.MISSION_ITEM_REACHED, seq=seq, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive mission item reached timed out")
            return self.TIMEOUT_ERROR
        elif message.get('seq') is not None:
            self.logger.debug(f"[Controller] Received mission item reached: {message['seq']}")
            return message['seq'] if message['seq'] is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for mission item reached")
            return self.BAD_RESPONSE_ERROR

    def send_clear_mission(self) -> int:
        """
        Clear the mission on ardupilot.

        Returns:
            int: 0 if the mission was cleared successfully
        """
        self.master.waypoint_clear_all_send() # type: ignore
        self.logger.info("[Controller] Sent clear mission")
        return 0

    async def set_mode(self, mode: str) -> int:
        """
        Set the ardupilot mode.

        Args:
            mode (str): The mode to set ardupilot to. Options include: "AUTO", "GUIDED", "FBWA", etc...

        Returns:
            int: 0 if the mode was set successfully, 111 if the mode is unknown, 101 if the response timed out.
        """
        if mode not in self.master.mode_mapping(): # type: ignore
            self.logger.error(f"[Controller] Unknown mode: {mode}")
            return self.UNKNOWN_MODE

        mode_id = self.master.mode_mapping()[mode] # type: ignore
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1
            mode_id,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_SET_MODE, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Set mode command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for set mode")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Set mode to {mode}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to set mode: {MAVResult.string(res)}")
            return res
            

    async def arm(self, force=False) -> int:
        """
        Arm ardupilot

        Args:
            force (bool): If True, ardupilot will be armed regardless of its state.

        Returns:
            int: 0 if ardupilot was armed successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            1,  # param1
            21196 if force else 0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Arm command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for arm")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info("[Controller] Armed")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to arm: {MAVResult.string(res)}")
            return res

    async def disarm(self, force=False) -> int:
        """
        Disarm ardupilot.

        Args:
            force (bool): If True, ardupilot will be disarmed regardless of its state.

        Returns:
            int: 0 if ardupilot was disarmed successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            0,  # param1
            21196 if force else 0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Disarm command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for disarm")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info("[Controller] Disarmed")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to disarm: {MAVResult.string(res)}")
            return res

    async def enable_geofence(self) -> int:
        """
        Enable the geofence.

        Returns:
            int: 0 if the geofence was enabled successfully, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,  # command
            0,  # confirmation
            1,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Enable geofence command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for enable geofence")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info("[Controller] Geofence enabled")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to enable geofence: {MAVResult.string(res)}")
            return res

    async def disable_geofence(self, floor_only=False) -> int:
        """
        Disable the geofence.

        Args:
            floor_only (bool): If True, only the floor of the geofence will be disabled.

        Returns:
            int: 0 if the geofence was disabled successfully, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,  # command
            0,  # confirmation
            2 if floor_only else 0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Disable geofence command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for disable geofence")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info("[Controller] Geofence disabled")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to disable geofence: {MAVResult.string(res)}")
            return res

    async def set_home(self, home_coordinate=Coordinate(0, 0, 0)) -> int:
        """
        Set the home location.

        Args:
            home_coordinate (Coordinate): The home coordinate to set. If omitted or (0, 0, 0), the current GPS location will be used.

        Returns:
            int: 0 if the home location was set successfully, error code if there was an error, 101 if the response timed out.
        """

        # if alt is 0, use the current altitude
        if home_coordinate.altitude_m == 0:
            current_pos = await self.receive_gps()
            home_coordinate.altitude_m = current_pos.altitude_m if isinstance(current_pos, Coordinate) else 0
        else:
            home_coordinate.altitude_m = home_coordinate.altitude_m

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            0,  # frame - MAV_FRAME_GLOBAL
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # current
            0,  # auto continue
            1 if home_coordinate == Coordinate(0,0,0) else 0,  # param1 - 1 for use current if coordinate is 0s
            0,  # param2
            0,  # param3
            0,  # param4
            home_coordinate.latitude_deg,  # param5
            home_coordinate.longitude_deg,  # param6
            home_coordinate.altitude_m,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_SET_HOME, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Set home command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for set home")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Set home to {home_coordinate}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to set home: {MAVResult.string(res)}")
            return res

    async def set_servo(self, servo_number: int, pwm: int) -> int:
        """
        Set the a servo to a specified PWM value.

        Args:
            servo_number (int): The servo number to set.
            pwm (int): The PWM value to set the servo to.

        Returns:
            int: 0 if the servo was set successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            servo_number,  # param1
            pwm,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error(f"[Controller] Set servo {servo_number} command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error(f"[Controller] Bad response received for set servo {servo_number}")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Set servo {servo_number} to pwm {pwm}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to set servo {servo_number}: {MAVResult.string(res)}")
            return res
    
    async def receive_channel_input(self) -> int | Any:
        """
        Wait for an RC_CHANNELS message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if an RC_CHANNELS message was received, 101 if the response timed out
        """

        message = await self.receive_message(MAVMessage.RC_CHANNELS)
        if message is None:
            self.logger.error("[Controller] Receive channel input timed out")
            return self.TIMEOUT_ERROR
        elif message.get('chancount') is not None:
            self.logger.debug(f"[Controller] Received channel input from {message['chancount']} channels")
            return message
        else:
            self.logger.error("[Controller] Bad response received for channel input")
            return self.BAD_RESPONSE_ERROR

    async def receive_wind(self, normalize_direction:bool=False) -> int | Any:
        """
        Wait for a WIND message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.
            normalize_direction (bool, optional) Flag to make wind direction relative to ground rather than aircraft. This will be an estimate since it requires two messages. Defaults to false

        Returns:
            response if a WIND message was received, 101 if the response timed out
        """

        wind_message = await self.receive_message(MAVMessage.WIND)
        if wind_message is None:
            self.logger.error("[Controller] Receive wind data timed out")
            return self.TIMEOUT_ERROR

        if normalize_direction:
            gps_message = await self.request_message(MAVMessage.GLOBAL_POSITION_INT)
            if gps_message is None:
                self.logger.error("[Controller] Failed to get GPS data for normalizing wind")
                return self.BAD_RESPONSE_ERROR
            wind_message['direction'] = (wind_message['direction'] - (gps_message['hdg'] / 100)) % 360


        self.logger.debug("[Controller] Received wind data")
        return wind_message

    async def receive_gps(self, timeout=TIMEOUT_DURATION, normalize_time=False) -> int | Coordinate:
        """
        Wait for a GLOBAL_POSITION_INT message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.
            normalize_time (bool): If True, the timestamp will be normalized to the controller's clock. Default is False.

        Returns:
            Coordinate: A Coordinate object containing the GPS data if received, TIMEOUT_ERROR (101) if the response timed out.
        """

        message = await self.receive_message(MAVMessage.GLOBAL_POSITION_INT, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive GPS data timed out")
            return self.TIMEOUT_ERROR
        elif message.get('lat') is not None and message.get('lon') is not None and message.get('alt') is not None and message.get('hdg') is not None:
            self.logger.debug(f"[Controller] Received GPS data: {message['lat']}, {message['lon']}, {message['alt']}, {message['hdg']}")
            if message is not None:

                if normalize_time and 'time_boot_ms' in message and self.offset_ns is not None:
                    # normalize the timestamp using the offset
                    normalized_timestamp = message['time_boot_ms'] * 1e6 - self.offset_ns

                else:
                    normalized_timestamp = message['time_boot_ms'] * 1e6  # convert to nanoseconds
                self.logger.debug(f"[Controller] GPS Timestamp (system): {message["time_boot_ms"]}")
                return Coordinate.from_int(
                    latitude_degE7=message['lat'],
                    longitude_degE7=message['lon'],
                    altitude_mm=message['relative_alt'],
                    heading_cdeg=message['hdg'],
                    timestamp_ms=normalized_timestamp
                )
            
            return self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for GPS data")
            return self.BAD_RESPONSE_ERROR

    async def receive_landing_status(self, timeout=TIMEOUT_DURATION) -> int:
        """
        Wait for a landed_state message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: The landing state if received, TIMEOUT_ERROR (101) if the response timed out, 0 if the state is undefined, 1 if on ground, 2 if in air, 3 if taking off, 4 if landing.
        """
        message = await self.receive_message(MAVMessage.EXTENDED_SYS_STATE, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive landing status timed out")
            return self.TIMEOUT_ERROR
        elif message.get('landed_state') is not None:
            self.logger.debug(f"[Controller] Received landing status: {MAVLandedState.string(message['landed_state'])}")
            return message['landed_state'] if message['landed_state'] is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for landing status")
            return self.BAD_RESPONSE_ERROR

    async def set_message_interval(self, message_type: MAVMessage, interval_us: int) -> int:
        """
        Set the message interval for the specified message type.

        Args:
            message_type (int): The type of message to set the interval for.
            interval_us (int): The interval in microseconds to set for the message type.
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: 0 if the message interval was set successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_type.value,  # param1
            interval_us,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Set message interval command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for set message interval")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Set interval for {message_type.name} to {interval_us} microseconds")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to set interval for {message_type.name}: {MAVResult.string(res)}")
            return res

    async def disable_message_interval(self, message_type: MAVMessage) -> int:
        """
        Disable the message interval for the specified message type.

        Args:
            message_type (str): The type of message to disable the interval for.
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: 0 if the message interval was disabled successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_type.value,  # param1
            -1,  # param2 # -1 disables the message
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Disable message interval command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for disable message interval")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Disabled interval for {message_type.name}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to disable interval for {message_type.name}: {MAVResult.string(res)}")
            return res

    async def receive_current_mission_index(self, seq: int) -> int:
        """
        Get the current mission index.

        Returns:
            int: The current mission index if received, TIMEOUT_ERROR (101) if the response timed out.
        """
        message = await self.receive_message(MAVMessage.MISSION_CURRENT, seq=seq)
        if message is None:
            self.logger.error("[Controller] Receive current mission index timed out")
            return self.TIMEOUT_ERROR
        elif message.get('seq') is not None:
            self.logger.debug(f"[Controller] Current mission index: {message['seq']}")
            return message['seq'] if message['seq'] is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Bad response received for mission item reached")
            return self.BAD_RESPONSE_ERROR

    async def set_current_mission_index(self, index: int, reset: bool = False) -> int:
        """
        sets the target mission index to the specified index

        Args:
            index (int): The index to set as the current mission index.
            reset (bool): Reset the mission, defaults to false
        Returns:
            int: 0 if the current mission index was set successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,  # command
            0,  # confirmation
            index,  # param1
            1 if reset else 0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Set mission index command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for set mission index")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Set mission index to {index}{' and reset' if reset else ''}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to set mission index: {MAVResult.string(res)}")
            return res

    async def start_mission(self, start_index, end_index) -> int:
        """
        Start the mission at the specified index.

        Args:
            start_index (int): The index to start the mission from.
            end_index (int): The index to end the mission at.

        Returns:
            int: 0 if the mission was started successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_MISSION_START,  # command
            0,  # confirmation
            start_index,  # param1
            end_index,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_MISSION_START, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Start mission command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for set mission index")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Started mission from waypoints {start_index} to {end_index}")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to start mission: {MAVResult.string(res)}")
            return res

    async def receive_attitude(self, timeout=TIMEOUT_DURATION) -> int | Any:
        """
        Wait for an attitude message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if an attitude message was received, TIMEOUT_ERROR (101) if the response timed out.
        """
        message = await self.receive_message(MAVMessage.ATTITUDE, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive attitude data timed out")
            return self.TIMEOUT_ERROR
        elif message.get('roll') is not None and message.get('pitch') is not None and message.get('yaw') is not None:
            self.logger.debug(f"[Controller] Received attitude data: roll={message['roll']}, pitch={message['pitch']}, yaw={message['yaw']}")
            return message
        else:
            self.logger.error("[Controller] Bad response received for attitude data")
            return self.BAD_RESPONSE_ERROR

    def request_timesync(self, current_time: int) -> int:
        """
        Request a timesync message from ardupilot.

        Args:
            current_time (int): The current time in nanoseconds.

        Returns:
            int: 0 if the timesync request was sent successfully.
        """
        self.master.mav.timesync_send( # type: ignore
            0,  # tc1 - set to 0 for request
            int(current_time),  # ts1 - current time in nanoseconds,
        )
        return 0

    async def receive_timesync(self, seq: int, timeout=TIMEOUT_DURATION) -> int | Any:
        """
        Wait for a timesync message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if a timesync message was received, TIMEOUT_ERROR (101) if the response timed out.
        """
        message = await self.receive_message(MAVMessage.TIMESYNC, seq=seq, timeout=timeout)
        if message is None:
            self.logger.error("[Controller] Receive timesync data timed out")
            return self.TIMEOUT_ERROR
        elif message.get('tc1') is not None and message.get('ts1') is not None:
            return message
        else:
            self.logger.error("[Controller] Bad response received for timesync data")
            return self.BAD_RESPONSE_ERROR
        
    async def sync_clocks(self) -> int:
        """
        Sync the flight controller clock with the system clock.

        Returns:
            int: 0 if the clocks were synced successfully, otherwise an error code.
        """
        self.logger.debug("[Flight] Syncing clocks...")
        ALPHA = 0.5
        NUM_SAMPLES = 10
        i = 0
        error_count = 0

        rtt_samples = []
        offset_samples = []
        while i < NUM_SAMPLES:
            ts1 = self.monotonic_time_ns()
            next_seq = self.__message_seq_by_type["TIMESYNC"] + 1
            self.request_timesync(ts1)

            response = await self.receive_timesync(next_seq)
            ts4 = self.monotonic_time_ns()

            if isinstance(response, int):
                self.logger.error("[Controller] Failed to sync clocks.")
                return response

            if response['ts1'] != ts1:
                self.logger.debug("[Controller] Received unintended timestamp.")
                error_count += 1
                if error_count > 10:
                    self.logger.error("[Controller] Too many unintended timestamps, canceling timesync.")
                    return -1
                continue

            rtt = ts4 - ts1
            offset = response['tc1'] - (ts1 + rtt // 2)

            # add to rolling samples
            self.local_samples.append(ts4)
            self.peer_samples.append(response['tc1'])
            if len(self.local_samples) > self.ROLLING_WINDOW:
                self.local_samples.pop(0)
                self.peer_samples.pop(0)
            
            rtt_samples.append(rtt)
            offset_samples.append(offset)

            i += 1

        rtt = sum(rtt_samples) / len(rtt_samples)
        offset = sum(offset_samples) / len(offset_samples)
        self.rtt_ns = rtt
        if self.offset_ns:
            self.offset_ns = (1 - ALPHA) * self.offset_ns + ALPHA * offset
        else: 
            self.offset_ns = offset

        timesync_update = Message(
            topic=f"{self.message_topic}_timesync_update" if self.message_topic else "timesync_update", 
            header={
                "offset": self.offset_ns,
                "rtt": self.rtt_ns,
            }
        )
        await self.msg_queue.put(timesync_update)

        self.last_sync_time = time.time()
        self.logger.debug(f"[Controller] Clocks synced successfully. RTT: {self.rtt_ns}, Offset: {self.offset_ns}")
        return 0
    
    async def clock_synchronizer(self):
        """
        Periodically sync the flight controller clock with the system clock.
        """
        while self.__running.is_set():
            await self.sync_clocks()
            await asyncio.sleep(self.CLOCK_SYNC_INTERVAL)

    def monotonic_time_ns(self) -> int:
        """Get the current monotonic time in nanoseconds since the controller started.

        Returns:
            int: The current monotonic time in nanoseconds since the controller started.
        """
        return time.monotonic_ns() - self.start_time
    
    def time_boot_ms(self) -> int:
        """Get the estimated system time of the connected vehicle since boot. Uses calculated timesync offset.

        Returns:
            int: The approximate time since boot for the connected vehicle. If a timesync offset has not been calculated, returns -1.
        """
        if self.offset_ns is None:
            return -1
        
        return int((self.monotonic_time_ns() + self.offset_ns) / 1e6)
    
    def get_message_seq(self, message_type: str) -> int:
        """Get the latest sequence number for a given message type

        Args:
            message_type (str): Message type name

        Returns:
            int: The latest sequence number
        """
        return self.__message_seq_by_type[message_type]
    
    async def send_reposition(
            self, 
            position: Coordinate, 
            radius_m: float = 0, 
            speed_mps: float = -1,
            yaw_mode: RepositionYawMode = RepositionYawMode.NONE,
            change_mode: bool = False,
            relative_yaw: bool = False
    ) -> int:
        """Send a guided MAV_DO_REPOSITION message

        Args:
            position (Coordinate): The destination coordinate.
            radius_m (float, optional): Loiter radius in meters. If omitted or 0, default is used.
            speed_mps (float, optional): Speed to travel at in m/s. If omitted or -1, default is used.
            yaw_mode (RepositionYawMode, optional): Mode for reposition loiter direction for planes. Defaults to USE_YAW for VTOL craft.
            change_mode (bool, optional): Flag to automatically change mode to guided upon message send. Defaults to False.
            relative_yaw (bool, optional): Flag to set Yaw relative to the vehicle current heading. If false, yaw relative to North. Defaults to False.
        """

        # bitwise operation for flags bitmask
        flags = (
            (1 if change_mode else 0)
            | (2 if relative_yaw else 0)
        )

        if yaw_mode == RepositionYawMode.NONE:
            param4 = float('nan')
        elif yaw_mode == RepositionYawMode.USE_YAW:
            param4 = position.heading_rad
        else:
            param4 = yaw_mode.value

        message = self.master.mav.command_int_encode(
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
            2,  # current (unused)
            1,  # auto continue (unused)
            speed_mps,  # param1
            flags,  # param2
            radius_m,  # param3
            param4,  # param4            
            position.latitude_degE7,  # x
            position.longitude_degE7,  # y
            position.altitude_m,  # z
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_DO_REPOSITION, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Reposition command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for send reposition")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Sent reposition to {position.latitude_deg}°, {position.longitude_deg}°; {position.altitude_m}m")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to reposition: {MAVResult.string(res)}")
            return res

    async def send_takeoff(
        self, 
        altitude_m: float,
        pitch_deg: float = 0, 
        require_horizontal_position: bool = True
    ) -> int:
        """Send a takeoff message

        Args:
            altitude_m (float): Altitude to ascend to
            pitch_deg (float, optional): Pitch to ascend at (planes only). Minimum pitch if airspeed sensor present, else desired pitch.
            require_horizontal_position (bool, optional): Require autopilot to have control over its horizontal position. Defaults to True.

        Returns:
            int: 0 for success, error code on failure
        """
        
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            0,  # confirmation
            pitch_deg,  # param1
            0,  # param2
            0 if require_horizontal_position else 1,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            altitude_m,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Takeoff command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for takeoff")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info(f"[Controller] Sent takeoff to {altitude_m} meters")
            return 0
        else:
            self.logger.error(f"[Controller] Failed to takeoff: {MAVResult.string(res)}")
            return res
    

    def release_rc(self, channel: int) -> int:
        """Disable RC override for a channel

        Args:
            channel (int): Channel number 1-8 to set. Values outside [1,8] will be rejected

        Returns:
            int: 0 if the message is sent, BAD_RESPONSE_ERROR for invalid input
        """
        return self.override_rc(channel, 0)

    def override_rc(self, channel: int, pwm: int) -> int:
        """Manually set RC PWM value for a specific channel, overriding RC receiver input

        Args:
            channel (int): Channel number 1-8 to set. Values outside [1,8] will be rejected
            pwm (int): PWM value to set from 1000 (low) to 2000 (high) microseconds. A value of 0 will release control of the RC channel back to the receiver. Nonzero values outside [1000,2000] will be rejected.

        Returns:
            int: 0 if the message is sent, BAD_RESPONSE_ERROR for invalid input
        """
        if (pwm < 1000 or pwm > 2000) and not pwm == 0:
            self.logger.error("[Controller] Invalid pwm value for set RC")
            return self.BAD_RESPONSE_ERROR
        elif channel < 1 or channel > 8:
            self.logger.error("[Controller] Invalid channel value for set RC")
            return self.BAD_RESPONSE_ERROR

        channels = [0] * 8
        channels[channel - 1] = pwm

        message = mavutil.mavlink.MAVLink_rc_channels_override_message(
            0, # target_system
            0, # target_component
            channels[0], # chan1_raw
            channels[1], # chan2_raw
            channels[2], # chan3_raw
            channels[3], # chan4_raw
            channels[4], # chan5_raw
            channels[5], # chan6_raw
            channels[6], # chan7_raw
            channels[7], # chan8_raw
        )

        self.send_message(message)
        if pwm == 0:
            self.logger.info(f"[Controller] Released RC channel {channel} control back to receiver")
        else:
            self.logger.info(f"[Controller] Set RC channel {channel} to PWM {pwm} microseconds") 
        return 0
    
    async def run_prearm_checks(self) -> int:
        """Run prearm checks at request.

        Returns:
            int: 0 on success, error code on failure.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,  # command
            0,  # confirmation
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        res = await self.send_command_with_ack(message, mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, self.TIMEOUT_DURATION)

        if res == self.TIMEOUT_ERROR:
            self.logger.error("[Controller] Run prearm checks command timed out")
            return self.TIMEOUT_ERROR
        elif res == self.BAD_RESPONSE_ERROR:
            self.logger.error("[Controller] Bad response received for run prearm checks")
            return self.BAD_RESPONSE_ERROR
        elif res == 0:
            self.logger.info("[Controller] Prearm checks started")
            return 0
        else:
            self.logger.error(f"[Controller] Prearm checks failed to run: {MAVResult.string(res)}")
            return res