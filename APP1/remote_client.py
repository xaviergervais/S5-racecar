# RemoteClient
# Connects to ROSMonitor on port 65432 through a TCP/IP socket
# Sends commands to ROSMonitor and prints the response
# Team 12 - Edouard Barrimo, Xavier Gervais, Sami Ghoul-Duclos, Étienne Renaud

import socket
from struct import *

def main():
    # Prompt for the IP address of the ROSMonitor
    HOST = input("Enter the IP address of the ROSMonitor: ")
    PORT = 65432  # The PORT on which the ROSMonitor is listening

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        #Set the socket as reusable and bind it to the appropriate IP/port
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")

        while True:
            # Prompt for the command
            command = input("Enter command (RPOS, OBSF, RBID) or 'exit' to quit: ").strip()
            if command.lower() == 'exit':
                break

            # Send the command to the ROSMonitor
            s.sendall(command.encode())

            # Receive the response from the ROSMonitor
            data = s.recv(1024)
            print(f"Received: {data.decode()}")
        
        # Close the connection
        s.close()

if __name__ == "__main__":
    main()
