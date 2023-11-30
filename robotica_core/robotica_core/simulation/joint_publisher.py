import zmq
import time
import pickle

def publish_joint_data(trajectory):
    # joint_data (list): list of joints 
    # TODO: make this configurable for multiple joints 
    # Or make specific data strucutre that can be dynamically created based on yml file 

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://127.0.0.1:5555")

    message = pickle.dumps(trajectory)

    socket.send(message)
    print("sent message")

    reply_message = socket.recv_string()
    print(f"Received reply: {reply_message}")

    # for data in trajectory:
    #     theta1, theta2 = data

    #     joint_data = f"{theta1} {theta2}"
    #     print(joint_data)
    #     socket.send_string(f"joint_data {joint_data}")
    #     time.sleep(1)

