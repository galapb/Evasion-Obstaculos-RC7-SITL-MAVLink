import struct
from utils.Enums import (
    AuthorityOverDrone,
    ActiveCamera,
    DeployPosition,
    StoreLoadType
)

class GCSMessage:
    """
    Represents a message received from the GCS UDP sender.
    Unpacks and stores fields according to defined enums.
    """

    def __init__(self, authority, active_camera, neural_network_active,
                 deploy_positions_12, deploy_positions_34,
                 store_1, store_2, store_3, store_4,
                 apply_joystick_exponential, investigate_point):
        self.authority = AuthorityOverDrone(authority)
        self.active_camera = ActiveCamera(active_camera)
        self.neural_network_active = bool(neural_network_active)
        self.deploy_positions_12 = DeployPosition(deploy_positions_12)
        self.deploy_positions_34 = DeployPosition(deploy_positions_34)
        self.store_1 = StoreLoadType(store_1)
        self.store_2 = StoreLoadType(store_2)
        self.store_3 = StoreLoadType(store_3)
        self.store_4 = StoreLoadType(store_4)
        self.apply_joystick_exponential = bool(apply_joystick_exponential)
        self.investigate_point = bool(investigate_point)


    @classmethod
    def from_bytes(cls, data: bytes):
        """
        Decodes 36 bytes (11 * int32) into a structured GCSMessage.
        """
        if len(data) != 44:
            raise ValueError("Expected 8 bytes for GCSMessage.")
        unpacked = struct.unpack('<11i', data)
        return cls(*unpacked)

    def __repr__(self):
        return (f"<GCSMessage authority={self.authority.name}, active_camera={self.active_camera.name}, "
                f"neural_network_active={self.neural_network_active}, "
                f"deploy_12={self.deploy_positions_12.name}, deploy_34={self.deploy_positions_34.name}, "
                f"store1={self.store_1.name}, store2={self.store_2.name}, "
                f"store3={self.store_3.name}, store4={self.store_4.name}>, "
                f"apply_joystick_exponential={self.apply_joystick_exponential}, "
                f"investigate_point={self.investigate_point}>")

    def as_dict(self):
        """
        Returns the decoded message as a dictionary.
        """
        return {
            "authority": self.authority,
            "active_camera": self.active_camera,
            "neural_network_active": self.neural_network_active,
            "deploy_positions_12": self.deploy_positions_12,
            "deploy_positions_34": self.deploy_positions_34,
            "store_1": self.store_1,
            "store_2": self.store_2,
            "store_3": self.store_3,
            "store_4": self.store_4,
            "apply_joystick_exponential": self.apply_joystick_exponential,
            "investigate_point": self.investigate_point,
        }
