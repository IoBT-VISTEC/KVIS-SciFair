from threading import Thread
import threading
import paho.mqtt.client as mqtt
from roslibpy import Ros, Topic, Message
import time

class Muse_app:
    def __init__(self):
        self.client = mqtt.Client()
        self.client2 = mqtt.Client()
        self.ros_client = Ros(host='10.205.240.34', port=9090)  
        self.publisher = Topic(self.ros_client, '/chatter', 'std_msgs/String')
        self.muse_left = "DC11"
        self.muse_right = "C00E"
        self.status_left = False
        self.status_right = False
        self.action_mode = 0

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        t = threading.Thread(target=self.worker)
        t.start()

        self.client2.on_connect = self.on_connect2
        self.client2.on_message = self.on_message
        self.client2.connect("localhost", 1883, 60)
        t2 = threading.Thread(target=self.worker2)
        t2.start()

        self.ros_client.on_ready(self.subscribing, run_in_thread=True)
        t3 = threading.Thread(target=self.worker3)
        t3.start()
        while True:
            pass

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe("topic/" + self.muse_left)
        #self.client.subscribe("topic/test")

    def on_connect2(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client2.subscribe("topic/" + self.muse_right)


    def worker(self):
        self.client.loop_forever()
        return

    def worker2(self):
        self.client2.loop_forever()
        return

    def on_message(self, client, userdata, message):
        print("message received " ,str(message.payload.decode("utf-8")))
        print("message topic=",message.topic)
        if message.topic == "topic/" + self.muse_left:
            if(message.payload.decode("utf-8") == '0'):
                self.status_left = False
            elif(message.payload.decode("utf-8") == '1'):
                self.status_left = True
            print("left = " , self.status_left)
        if message.topic == "topic/" + self.muse_right:
            if(message.payload.decode("utf-8") == '0'):
                self.status_right = False
            elif(message.payload.decode("utf-8") == '1'):
                self.status_right = True
            print("right = " , self.status_right)
        # print("message qos=",message.qos)
        # print("message retain flag=",message.retain)

    def subscribing(self):
        print("subscribe to Ros..")
        self.publisher.subscribe(self.receive_message)
        

    def uninitialize(self):
        # nop
        return

    def receive_message(self, message):
        # context['counter'] += 1
        print('Receive data from Ros, ' , message['data'])
        #assert message['data'] == 'hello world', 'Unexpected message content'
        if(message['data'] == "action"):
            self.client.publish("topic/parameters", 'action')
            time.sleep(3)
            print("lastest status from left = " , self.status_left)
            print("lastest status from right = " , self.status_right)
            if self.status_left and not self.status_right: ##======================TURN LEFT
                self.action_mode = 1
            elif not self.status_left and self.status_right: ##======================TURN RIGHT
                self.action_mode = 2
            elif not self.status_left and not self.status_right: ##======================MOVE FORWARD
                self.action_mode = 3
            elif self.status_left and self.status_right: ##======================MOVE BACKWARD
                self.action_mode = 4
            print("summary action mode = ", str(self.action_mode))
            self.client.publish("topic/parameters", 'wait')
            isConnected = self.ros_client.is_connected
            print("isConnected = ", isConnected)
            if(isConnected):
                self.publisher.publish(Message({'data': str(self.action_mode)}))

    def worker3(self):
        self.ros_client.run_forever()	

if __name__ == "__main__":
    Muse_app()