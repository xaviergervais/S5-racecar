# RemoteClient
# Connects to ROSMonitor on port 65432 through a TCP/IP socket
# Sends commands to ROSMonitor and prints the response
# Team 12 - Edouard Barrimo, Xavier Gervais, Sami Ghoul-Duclos, Étienne Renaud

import socket
from struct import *

RPOS_FORMAT = "fffxxxx"
OBSF_FORMAT = "Ixxxxxxxxxxxx"
RBID_FORMAT = "Ixxxxxxxxxxxx"

def int2ip(ip):
    # Converts a 32-bit unsigned integer to an IP address.
    return socket.inet_ntoa(pack("!I", ip))

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

            # Decode the response
            if command.lower() == 'rpos':
                x, y, theta = unpack(RPOS_FORMAT, data)
                message = f"Received message: x = {x}, y = {y}, theta = {theta}"
            elif command.lower() == 'obsf':
                obstacle = unpack(OBSF_FORMAT, data)
                message = f"Received message: obstacle = {obstacle}"
            elif command.lower() == 'rbid':
                ip_int = unpack(RBID_FORMAT, data)
                ip_str = int2ip(ip_int)
                message = f"Received message: id = {ip_str}"
            
            # Print the response
            print(message)
        
        # Close the connection
        s.close()

if __name__ == "__main__":
    main()
