from __future__ import print_function
import roslibpy
import json
from threading import Thread
import socket
import time

sum_network_latency = 0.0
avg_network_latency = 0.0
count = 0

def parse_message(message):
    global count
    global sum_network_latency
    global avg_network_latency

    listener_start = time.time()

    parsed_message = json.loads(message)
    message_id = parsed_message['message_id']
    fused_pose = parsed_message['fused_pose']
    time1 = parsed_message['time1']

    count += 1
    network_latency = time.time() - time1
    sum_network_latency += network_latency
    avg_network_latency = sum_network_latency / count

    with open('data', 'w') as file:
        file.write(str(time1)+":"+str(fused_pose))  # Convert list to string and write it to the file
    with open('t1', 'w') as file:
        file.write(str(time.time()))
    with open('msg_id', 'w') as file:
        file.write(str(message_id))

    # Log received IDs
    # with open('received_ids', 'a') as file:
    #     file.write(f"{message_id}\n")

    print(f"Received: ID {message_id}, Latency: {round(network_latency*1000, 4)}ms, Avg: {round(avg_network_latency*1000, 4)}ms")

def listen_from_xr(xr, port):
    client = roslibpy.Ros(host=xr, port=port)
    client.run()

    listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
    listener.subscribe(lambda message: parse_message(message['data']))

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()

def robot_to_xr(robot, port, xr, period=1):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((xr, port))
        while True:
            data = f"Test data from robot"
            s.sendall(data.encode())
            time.sleep(period)

if __name__ == '__main__':
    host = "10.13.145.123"
    port = 9090

    listen_thread = Thread(target=listen_from_xr, args=(host, port))
    # send_thread = Thread(target=robot_to_xr, args=("", 9091, "localhost", 0.05))  # 50ms period

    listen_thread.start()
    # send_thread.start()

    listen_thread.join()
    # send_thread.join()
