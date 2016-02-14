"""A basic script to store points from a cb2 robot

Basic Usage: Run the script, with commandline args. Press `c` to capture  a
point. Press `s` to save and exit.

Copyright (c) 2016 GTRC. All rights reserved.
"""

import socket
import time
import argparse
import cb2_receive
import json
import sys
from contextlib import closing


def main():
    # Parse in arguments
    parser = argparse.ArgumentParser(
        description='Save Points',
        epilog="This software is designed to save points from a cb2 robot to a"
               " file for future use. Points are saved as both joint and"
               " cartesian points to the designated file. Once the program is "
               "running, press [c] to capture a point and press [s] to save"
               " the file and exit.")

    parser.add_argument("-f", "--file", metavar="file", type=str,
                        help='The file to save data to.',
                        default="cb2points.json")

    parser.add_argument("--ip", metavar="ip", type=str,
                        help='IP address of the robot', default="192.168.1.100")

    parser.add_argument("--port", metavar="port", type=int,
                        help='IP port on the robot', default=30003)

    args = parser.parse_args()

    # Check to make sure that we can access the file
    try:
        f = open(args.file, 'w')
        print "Able to access file, closing it now until we are ready for it."
    except IOError:
        print "Unable to access file, bailing out."
        sys.exit("Unable to access file")
    f.close()

    host = args.ip    # The remote host
    port = args.port  # The same port as used by the server

    print 'trying to connect to:    {}:{}'.format(host, port)
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM))\
            as robot_socket:
        robot_socket.connect((host, port))
        with cb2_receive.URReceiver(robot_socket, False) as my_ur_receiver:
            my_ur_receiver.start()
            run = True
            point_count = 0
            json_dict = dict()
            json_dict['time'] = time.time()
            json_dict['points'] = dict()
            while run:
                try:
                    key_input = str(raw_input('Input:'))
                except ValueError:
                    print ("Input was not valid. You must input either [c] or"
                           " [s]")
                    key_input = None
                if key_input is not None:
                    if key_input in ('c', 'C'):
                        with my_ur_receiver.lock:
                            json_dict['points'][point_count] = \
                                {'cartesian': my_ur_receiver.position,
                                 'joint': my_ur_receiver.actual_joint_positions}
                            point_count += 1
                    elif key_input in ('s', 'S'):
                        run = False
                    else:
                        print ("Input was not valid. You must input either [c]"
                               " or [s]")
            with open(args.file, 'w') as f:
                json.dump(json_dict, f, indent=4)

if __name__ == "__main__":
    main()
