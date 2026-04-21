# mission_item.py
# version: 1.0.1
# Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2025-09-15
# Organization: PSU UAS

"""
An ardupilot mission item.
"""

from pymavlink import mavutil # type: ignore[import]

from MAVez.coordinate import Coordinate


class MissionItem:
    """
    Represents a mission item for ardupilot.

    Args:
        seq (int): Sequence number of the mission item.
        frame (int): Frame of reference for the mission item.
        command (int): Command to be executed.
        current (int): Whether this is the current mission item.
        auto_continue (int): Whether to automatically continue to the next item.
        coordinate (Coordinate): The coordinate for the mission item.
        type (int): Type of the mission item, default is 0.
        param1 (float): Parameter 1 for the command, default is 0.
        param2 (float): Parameter 2 for the command, default is 0.
        param3 (float): Parameter 3 for the command, default is 0.
        param4 (float): Parameter 4 for the command, default is 0.

    Returns:
        Mission_Item: An instance of the Mission_Item class.
    """

    def __init__(
        self,
        seq: int,
        frame: int,
        command: int,
        current: int,
        auto_continue: int,
        coordinate: Coordinate,
        type: int = 0,
        param1: float = 0,
        param2: float = 0,
        param3: float = 0,
        param4: float = 0,
    ):
        self.seq = int(seq)
        self.frame = int(frame)
        self.command = int(command)
        self.current = int(current)
        self.auto_continue = int(auto_continue)
        self.x = coordinate.latitude_degE7
        self.y = coordinate.longitude_degE7
        self.z = coordinate.altitude_m # mavlink says z should be in mm but ardupilot uses m
        self.param1 = float(param1)
        self.param2 = float(param2)
        self.param3 = float(param3)
        self.param4 = float(param4)
        self.type = type

    def __str__(self):
        return f"Seq: {self.seq} \nFrame: {self.frame}\nCommand: {self.command}\nCurrent: {self.current}\nAuto Continue: {self.auto_continue}\nX: {self.x}\nY: {self.y}\nZ: {self.z}\nType: {self.type}\nParam1: {self.param1}\nParam2: {self.param2}\nParam3 {self.param3}\nParam4 {self.param4}"

    __repr__ = __str__

    @property
    def message(self):
        """
        The MAVLink message for the mission item.

        Returns:
            mavutil.mavlink.MAVLink_mission_item_int_message: The MAVLink message for the mission item.
        """
        message = mavutil.mavlink.MAVLink_mission_item_int_message(
            0,  # target_system
            0,  # target_component
            self.seq,  # seq
            self.frame,  # frame
            self.command,  # command
            self.current,  # current
            self.auto_continue,  # auto continue
            self.param1,  # param1
            self.param2,  # param2
            self.param3,  # param3
            self.param4,  # param4
            self.x,  # x
            self.y,  # y
            self.z,  # z
            self.type,  # type
        )

        return message
