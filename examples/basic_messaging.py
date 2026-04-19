"""
A simple publisher example that publishes all messages received.

After starting this script, you can run the subscriber example to see the messages being published:
    python examples/basic_messaging.py
Then run this subscriber script:
    python examples/message_receiver.py
"""

from MAVez import flight_controller
from MAVez.safe_logger import configure_logging
import asyncio


logger = configure_logging()
async def main():

    # Using async context manager to handle messaging setup and teardown
    async with flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', 
                                                    baud=57600, 
                                                    logger=logger, 
                                                    message_host='127.0.0.1', 
                                                    message_port=5555, 
                                                    message_topic='mavlink') as controller:

        await controller.set_geofence("./examples/sample_missions/sample_fence.txt")

        await controller.arm()

        await controller.auto_mission_takeoff("./examples/sample_missions/sample1.txt")
        controller.append_mission("./examples/sample_missions/sample2.txt")
        controller.append_mission("./examples/sample_missions/sample3.txt")

        await controller.auto_send_next_mission()
        await controller.auto_send_next_mission()

        await controller.wait_for_landing()

        await controller.disarm()




if __name__ == "__main__":
    asyncio.run(main())

