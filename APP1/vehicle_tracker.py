# VehicleTracker
# Listens for UDP broadcast data on a specific port
# Team 12 - Edouard Barrimo, Xavier Gervais, Sami Ghoul-Duclos, Étienne Renaud

import socket
from struct import *

def main():
    # Prompt for the IP address of the PositionBroadcast
    HOST = input("Enter the IP address of the PositionBroadcast: ")
    PORT = 65431

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        #Set the socket as reusable and bind it to the appropriate IP/port
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))

        # Start listening for any broadcasted message
        print(f"Listening for broadcasted data on {HOST}:{PORT}")

        #Listen to data until the user interrupts the program
        try:
            while True:
                data, addr = s.recvfrom(1024)
                message = data.decode()
                print(f"Received message: {message} from {addr}")
        except KeyboardInterrupt:
            pass

        # Close the connection
        s.close()

if __name__ == "__main__":
    main()
