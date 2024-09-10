#!/usr/bin/env python3

import socket
from struct import *

#!/usr/bin/env python3

import socket

def main():
    # Prompt for the IP address of the ROSMonitor
    host = input("Enter the IP address of the ROSMonitor: ")
    port = 65432  # The port on which the ROSMonitor is listening

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        print(f"Connected to {host}:{port}")

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

if __name__ == "__main__":
    main()



