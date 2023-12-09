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

