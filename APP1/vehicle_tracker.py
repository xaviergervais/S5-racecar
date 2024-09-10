#!/usr/bin/env python

import socket
from struct import *

# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

def main():
    HOST = input("Enter the IP address of the PositionBroadcast: ")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        #s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        print(f"Listening for broadcasted data on {HOST}:{PORT}")
        while True:
            data, addr = s.recvfrom(1024)
            message = data.decode()
            print(f"Received message: {message} from {addr}")

if __name__ == "__main__":
    main()
