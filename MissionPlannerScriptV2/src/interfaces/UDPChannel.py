import socket

class UDPChannel:

    def __init__(self, ip, port, isReceiver=True):
        self.ip_ = ip
        self.port_ = port
        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if isReceiver:
            self.socket_.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket_.bind((ip, port))
    
    def Send(self, data, targetIp, targetPort):
        self.socket_.sendto(data, (targetIp, targetPort))
    
    def Receive(self, bufsize=1024):
        return self.socket_.recvfrom(bufsize)