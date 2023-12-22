import zmq
import threading
import pickle

class RoboticaPublisher:
    def __init__(self, port, topic):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        self.topic = topic

    def publish(self, data):
        #pickled_data = pickle.dumps(data)
        msg = { f"{self.topic}": data }
        self.socket.send_json(msg)
        #self.socket.send_string(f"{self.topic} {pickled_data}")

class RoboticaSubscriber:
    def __init__(self, port, topic):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{port}")
        #self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

        self.topic = topic

    def subscribe(self, callback):
        thread = threading.Thread(target=self._subscribe, args=(self.topic, callback))
        thread.daemon = True  # Set the thread as a daemon
        thread.start()

    def _subscribe(self, topic, callback):
        while True:
            msg = self.socket.recv_json()

            if topic in msg:
                data = msg[self.topic]
                callback(data)

class RoboticaService:
    def __init__(self, port):
        context = zmq.Context()
        self.data_socket = context.socket(zmq.REP)
        self.data_socket.bind(f"tcp://127.0.0.1:{port}")

        self.poller = zmq.Poller()
        self.poller.register(self.data_socket, zmq.POLLIN)

    def received_request(self, block_timeout):
        """
        Args:
            block_timeout (int): Time in ms to check for request
        """
        return self.poller.poll(block_timeout)
    
    def unpack_request(self):
        serialized_data = self.data_socket.recv()
        return pickle.loads(serialized_data)
    
    def send_response(self, data):
        self.data_socket.send(pickle.dumps(data))


class RoboticaClient:
    def __init__(self, port):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect(f"tcp://127.0.0.1:{port}")

    def send_req(self, data):
        message = pickle.dumps(data)
        self.socket.send(message)

        # Blocks until reply msg is received 
        reply_message = pickle.loads(self.socket.recv())
        return reply_message


