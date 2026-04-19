import asyncio

from MAVez import FlightController, Coordinate, enums
from MAVez.safe_logger import configure_logging


logger = configure_logging()
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

        await controller.set_message_interval(enums.MAVMessage.GLOBAL_POSITION_INT, 500_000)

        coord_1 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=10).offset_coordinate(200, 27)
        coord_2 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=10).offset_coordinate(300, 180)

        await controller.takeoff(10, 20)
        await controller.go_to(coord_1, 60, 2, loiter_radius_m=80)
        await controller.land_here(5, 60)

        await controller.takeoff(10, 20)
        await controller.go_to(coord_2, 60, 2, loiter_radius_m=80, relative_yaw=True)
        await controller.set_mode("RTL")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

