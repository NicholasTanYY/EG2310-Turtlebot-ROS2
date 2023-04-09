import paho.mqtt.client as mqtt
import rclpy
from std_msgs.msg import String

broker_address = "192.168.110.120"
topic = "TableNum"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topic)

def on_message(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("Message received on topic "+msg.topic+" with payload "+message)

    msg = String()
    msg.data = message
    pub.publish(msg)

def on_log(client, userdata, level, buf):
    print("Log: ", buf)
    
rclpy.init()
node = rclpy.create_node("mqtt_subscriber")
pub = node.create_publisher(String, "mqtt_data", 10)
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("nicholas", "EG2310")
client.on_log = on_log
client.enable_logger()
client.connect(broker_address)
client.loop_start()  # Start networking daemon

try:
    rclpy.spin(node)
finally:
    client.loop_stop()
    client.disconnect()
    node.destroy_node()
    rclpy.shutdown()