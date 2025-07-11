import threading
import logging
import subprocess
import sys
from ConfigLoader import ConfigLoader
from state.DroneState import DroneState
from interfaces.MavLinkMasterDroneReaderInterface import MavlinkMasterDroneReaderInterface
from interfaces.MavLinkMasterDroneWriterInterface import MavlinkMasterDroneWriterInterface
from pymavlink import mavutil


class MissionPlannerIntegrator:
    def __init__(self):

        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger("MissionPlanner")
        self.components = []  # List to store initialized components
        logging.info("Starting Mission Planner...")
        self.config_ = ConfigLoader("config.xml")
        self.state_ = DroneState()

        connection_string_master_ = f"udp:{self.config_.udpIpMavlinkMasterDrone_}:{self.config_.portSendMavlinkMasterDrone_}"
        self.master_ = mavutil.mavlink_connection(connection_string_master_)

        self.mavlinkMasterReaderInterface_ = MavlinkMasterDroneReaderInterface(self.state_,
                                                                               self.master_)

        self.mavLinkMasterWriterInterface_ = MavlinkMasterDroneWriterInterface(self.state_,
                                                                               self.master_)

    def InstallDependencies(self):
        try:
            logging.info("Installing dependencies...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
            logging.info("Dependencies installed successfully.")
        except subprocess.CalledProcessError as e:
            logging.error("Failed to install dependencies: " + str(e))
            sys.exit(1)

    def Start(self):

        self.logger.info("Starting Mission Planner Integration process")
        self.logger.info("Installing dependencies")
        self.InstallDependencies()
        self.logger.debug("Dependencies installed.")

        threads = []

        self.logger.info("Starting Mavlink Master Drone Reader Interface thread...")
        t_mav_master = threading.Thread(
            target=self.mavlinkMasterReaderInterface_.MavlinkReader,
            name="Mavlink Master Reader Interface Thread",
            daemon=False  # Set to False so that the thread runs as a non-daemon thread
        )
        t_mav_master.start()
        threads.append(t_mav_master)

        self.logger.info("All threads have been launched.")
        # Wait for all threads to finish
        for thread in threads:
            self.logger.info(f"Waiting for thread {thread.name} to finish...")
            thread.join()

        self.logger.info("All threads have terminated. Mission Planner Integration process completed.")


if __name__ == "__main__":
    mp = MissionPlannerIntegrator()
    mp.Start()
