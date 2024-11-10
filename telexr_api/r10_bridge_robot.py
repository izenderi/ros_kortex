#! /usr/bin/env python3
"""RosBridge"""

from __future__ import print_function
import roslibpy
import json
from threading import Thread

import rospy

import time

import socket

sum_network_latency = 0.0
avg_network_latency = 0.0
count = 0

data_string = ''
time1 = 0.0

def parse_message(message):
  global count
  global sum_network_latency
  global avg_network_latency
  global data_string
  global time1

  listener_start = time.time()

  # Parse the JSON data string
  parsed_message = json.loads(message)
  
  # Extract the values
  data_string = parsed_message['fused_pose']
  time1 = parsed_message['time1']

  # Print or process the extracted values
  # print(f"Received data: {data_string}, time1: {time1}")

  count += 1
  network_latency = time.time() - time1
  sum_network_latency += network_latency
  avg_network_latency = sum_network_latency / count
  
  # print(f"Netowkr Latency: {round(network_latency*1000, 4)}ms, Avg: {round(avg_network_latency*1000, 4)}ms")

  with open('data', 'w') as file:
    file.write(str(time1)+":"+str(data_string))  # Convert list to string and write it to the file
  with open('t1', 'w') as file:
    file.write(str(time.time()))

  listener_latency = time.time() - listener_start

  print(f"Netowkr Latency: {round(network_latency*1000, 4)}ms, Avg: {round(avg_network_latency*1000, 4)}ms, Listener Latency: {round(listener_latency*1000, 4)}ms")


def listen_from_xr(xr, port):

  client = roslibpy.Ros(host=xr, port=port)
  client.run()

  listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

  # Modify the lambda to parse JSON
  listener.subscribe(lambda message: parse_message(message['data']))

  try:
      while True:
          pass
  except KeyboardInterrupt:
      client.terminate()

def robot_to_xr(robot, port, xr, period=1):
    global time1
    global data_string

    # Function to send data back to xr_bridge
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((xr, port))  # Ensure it matches xr_bridge's listen port
        while True:
            data = f"{str(time1)}:{str(data_string)}"

            #==================<RTEN>======================
            time.sleep(0.2) # Maunal delay mimic single-way from robot to xr
            #==================<RTEN/>======================

            s.sendall(data.encode())
            time.sleep(period)

# if __name__ == '__main__':
#   try:
#     rospy.init_node('rosbridge_robot')

#     # host = "10.13.146.101"  # Replace with server IP address if needed
#     host = "localhost"  # Replace with server IP address if needed
#     port = 9090
#     listen_from_xr(host, port)
#   except rospy.ROSInterruptException:
#     print("program interrupted before completion")

if __name__ == '__main__':
    rospy.init_node('rosbridge_robot')
    host = "localhost"
    port = 9090

    listen_thread = Thread(target=listen_from_xr, args=(host, port))
    send_thread = Thread(target=robot_to_xr, args=("", 9091, "localhost", 0.05)) # 0.05 is 50ms period

    listen_thread.start()
    send_thread.start()

    listen_thread.join()
    send_thread.join()