from threading import Thread
import threading
import paho.mqtt.client as mqtt
# def runA(name):
#     while True:
#         print(name)

# def runB(name2):
#     while True:
#         print(name2)

# if __name__ == "__main__":
#     t1 = Thread(target = runA())
#     t2 = Thread(target = runB())
#     t1.setDaemon(True)
#     t2.setDaemon(True)
#     t1.start()
#     t2.start()
#     while True:
#         pass
client = mqtt.Client()

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

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

if __name__ == "__main__":
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    t = threading.Thread(target=worker)
    t.start()
    while True:
        pass