from MAVez.controller import Controller
from MAVez.safe_logger import configure_logging
from uas_messenger.subscriber import Subscriber
import asyncio
import logging
import time



async def main():
    logger = configure_logging(level=logging.DEBUG)
    controller = Controller(timesync=True, logger=logger)
    await controller.start()

    async def callback(telem):
        telem_time = telem.header["time_boot_ms"] * 1e6 - controller.offset_ns
        print(telem_time, time.monotonic_ns(), (telem_time-time.monotonic_ns()) / 1e6)

    sub = Subscriber(host="127.0.0.1", port=5555, topics=["GLOBAL_POSITION_INT"], callback=callback)
    sub.start()


    try:
        while True:
            await asyncio.sleep(1)
    finally:
        await controller.stop()
        await sub.close()

if __name__ == "__main__":
    asyncio.run(main())