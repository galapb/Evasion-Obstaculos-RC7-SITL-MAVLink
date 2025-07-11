import xml.etree.ElementTree as ET
import logging
import sys
import traceback
import os

def get_text(element):
    """Safely return element.text.strip() or an empty string."""
    return (element.text or "").strip()

class ConfigLoader:
    def __init__(self, filePath="config.xml"):
        self.filePath = filePath
        if not os.path.isfile(filePath):
            logging.error(f"Config file not found: {filePath}")
            sys.exit(1)

        try:
            logging.info("Loading configuration from XML...")
            # parse & root
            self.tree = ET.parse(filePath)
            self.root = self.tree.getroot()

            # Network
            network = self.root.find("Network")
            self.udpIpMavlinkMasterDrone_     = get_text(network.find("UdpIpMavlinkMasterDrone"))

            # Ports
            ports = self.root.find("Ports")
            self.portRecMavlinkMasterDrone_   = int(get_text(ports.find("PortRecMavlinkMasterDrone")))
            self.portSendMavlinkMasterDrone_  = int(get_text(ports.find("PortSendMavlinkMasterDrone")))


        except Exception as e:
            logging.error("Error loading XML configuration:", exc_info=True)
            sys.exit(1)

        logging.info("Configuration loaded successfully.")

    def get_rstp_paths(self):
        """Return a list of all child strings under <RSTPPaths>."""
        rstp = self.root.find("RSTPPaths")
        if rstp is None:
            return []
        # strip surrounding quotes if present
        return [ (child.text or "").strip().strip('"')
                 for child in rstp
                 if child.text and child.text.strip() ]
