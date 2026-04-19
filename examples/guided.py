import asyncio
import time
import logging

from MAVez import FlightController, Coordinate, enums
from MAVez.safe_logger import configure_logging


logger = configure_logging(logging.DEBUG)
async def main():
    controller = FlightController(
        connection_string='tcp:127.0.0.1:5762', 
        baud=57600, 
        logger=logger,
        message_host="127.0.0.1",
        message_port=5555,
        message_topic="",
        timesync=True,
        failsafe=True,
    )
    
    async with controller:

        await controller.set_mode("GUIDED")
        await controller.set_message_interval(enums.MAVMessage.GLOBAL_POSITION_INT, 500_000)
        await controller.arm()

        await controller.takeoff(10, 20, critical=False)

        coord_1 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=10)
        await controller.send_reposition(coord_1)
        await controller.wait_for_position_reached(coord_1, 2, 60)
        # await controller.go_to(coord_1, 60, 2, loiter_radius_m=80)

        # coord_2 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=10)
        # await controller.go_to(coord_2, 60, 2, loiter_radius_m=80, relative_yaw=True)

        # controller.override_rc(4, 2000)
        # await asyncio.sleep(2.5)
        # controller.release_rc(4)

        await controller.land_here(5, 60)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

