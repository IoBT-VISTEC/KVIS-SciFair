from threading import Thread
import threading
import paho.mqtt.client as mqtt
client = mqtt.Client()
import time

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("topic/parameters")
    #self.client.subscribe("topic/test")


# The callback for when a PUBLISH message is received from the server.

def worker():
    client.loop_forever()
    return

if __name__ == "__main__":
    client.on_connect = on_connect
    client.connect("localhost", 1883, 60)
    t = threading.Thread(target=worker)
    t.start()
    while True:
        client.publish("topic/parameters",'1')
        time.sleep(1)
        print('sending data')
        pass