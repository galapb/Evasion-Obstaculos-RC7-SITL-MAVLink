from utils.GCSMessage import DeployPosition
def DeployCommandToPWM(value):
    if(value==DeployPosition.None_):
        return 1500
    elif(value==DeployPosition.First):
        return 1900
    elif(value==DeployPosition.Second):
        return 1100
    return 1500