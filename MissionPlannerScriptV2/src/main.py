import os
    # ✅ Force MAVLink 2.0 globally to use RC_CHANNELS_OVERRIDE from ch9to16
os.environ["MAVLINK20"] = "1"
from MissionPlannerIntegrator import MissionPlannerIntegrator
from multiprocessing import Process
from interfaces.MavLinkMasterDroneReaderInterface import MavlinkMasterDroneReaderInterface
import logging
import sys
import time


# Set up the global logger for Mission Planner
logger = logging.getLogger("MissionPlanner")
logger.setLevel(logging.INFO)

# File handler: write logs to "mission_planner.log"
file_handler = logging.FileHandler("mission_planner.log")
file_handler.setLevel(logging.INFO)
file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

# Stream handler: output logs to the console (stdout)
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.INFO)
stream_formatter = logging.Formatter('%(name)s: %(levelname)s %(message)s')
stream_handler.setFormatter(stream_formatter)
logger.addHandler(stream_handler)

logger.info("Logger configured: output to console and file 'mission_planner.log'.")


def Main():
    mp = MissionPlannerIntegrator()
    mp.Start()

    # Create an instance of MavlinkMasterDroneReaderInterface.
    # This requires the shared state object and the MAVLink connection object
    # from the MissionPlannerIntegrator instance (mp).
    # Ensure that 'mp.state_' and 'mp.mavlink_connection_' are the correct attribute names
    # in your MissionPlannerIntegrator class that provide these objects.
    reader_interface_instance = MavlinkMasterDroneReaderInterface(
        state=mp.state_,  # Assumes mp exposes state as 'state_'
        mavlinkConnection=mp.mavlink_connection_  # Assumes mp exposes MAVLink connection as 'mavlink_connection_'
    )

    # PROCESS: lector de MAVLink
    reader_proceso = Process(
        target=reader_interface_instance.MavlinkReader
    )
    reader_proceso.start()

if __name__ == "__main__":
    Main()