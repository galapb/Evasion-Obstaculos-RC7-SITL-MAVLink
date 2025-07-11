from enum import IntEnum

class ActiveCamera(IntEnum):
    FPV = 0
    Thermal = 1


class AuthorityOverDrone(IntEnum):
    Manual = 0
    MetaQuestOperator = 1
    GcsOperator = 2


class DeployPosition(IntEnum):
    None_ = 0          # 'None' is a reserved keyword in Python
    First = 1
    Second = 2


class QMLElementType(IntEnum):
    Text = 0
    Image = 1
    None_ = 2          # Same as above — renamed from 'None'


class StoreLoadType(IntEnum):
    Empty = 0
    ChaserDrone = 1
    Gps = 2
