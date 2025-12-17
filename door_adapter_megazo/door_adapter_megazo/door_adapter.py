import sys
import yaml
import json
import time
import argparse
import threading

import rclpy
from door_adapter_megazo.DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rclpy.time import Time
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes 


###############################################################################

class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__('door_adapter')
        self.get_logger().info('Starting door adapter...')

        # Get value from config file
        self.door_name = config_yaml['door']['name']
        self.door_close_feature = config_yaml['door']['door_close_feature']
        self.door_signal_period = config_yaml['door']['door_signal_period']
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']
        
        url = config_yaml['door']['api_endpoint']
        mqtt_endpoint = config_yaml['door']['mqtt_endpoint']
        mqtt_port = config_yaml['door']['mqtt_port']
        mqtt_topic = config_yaml['door']['mqtt_topic']
        username = config_yaml['door']['username']
        password = config_yaml['door']['password']
        mqtt_password = config_yaml['door']['mqtt_password']
        mqtt_client_id = config_yaml['door']['mqtt_client_id']
        project_id = config_yaml['door']['project_id']
        ICED_id = config_yaml['door']['ICED_id']
        
        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']

        self.api = DoorClientAPI(url,username,password,project_id,ICED_id, self.get_logger())
       
        assert self.api.connected, "Unable to establish connection with door"
        
        # default door state - closed mode
        self.door_mode = DoorMode.MODE_CLOSED
        # open door flag
        self.open_door = False
        self.check_status = True #ensure initial door state is correct
        
        self.door_states_pub = self.create_publisher(
            DoorState, door_pub['topic_name'], 10)

        self.door_request_sub = self.create_subscription(
            DoorRequest, door_sub['topic_name'], self.door_request_cb, 10)

        self.periodic_timer = self.create_timer(
            self.door_state_publish_period, self.time_cb)

        #MQTT
        # Create the MQTT client instance
        client = mqtt.Client(client_id=mqtt_client_id, protocol=mqtt.MQTTv5, transport='websockets')
        client.username_pw_set(username,mqtt_password)
        
        # Set the callback functions
        client.on_connect=self.on_connect_mqtt
        client.on_disconnect=self.on_disconnect
        client.message_callback_add(mqtt_topic, self.on_message_mqtt)

        try:
            # Define user properties
            properties=Properties(PacketTypes.PUBLISH)
            properties.UserProperty=[("ProjectID",str(project_id))]

            # Connect to the broker via WebSocket with the user properties (MQTT v5)
            client.tls_set()
            client.ws_set_options(path="/mqtt")  # check broker docs
            client.connect(mqtt_endpoint, mqtt_port, 60, properties=properties)

        except Exception as e:
            self.get_logger().error(f"{e}")
            self.get_logger().error('MQTT connection fail')
            exit(1) #Should quit or raise flag to quit or retry

        #subscribe
        client.subscribe(mqtt_topic, qos=0)

        client.loop_start() #start the loop


    def on_message_mqtt(self, client, userdata, msg):
        #print(msg.payload)
        decoded_message=str(msg.payload.decode("utf-8"))
        msg_=json.loads(decoded_message)
        mqttEvent = msg_['doorEvent']['EventType']
        self.get_logger().info(f"New MQTT Event: = {mqttEvent}")
        self.get_logger().info(f"{msg_}")

        if mqttEvent == 3:
            self.door_mode = DoorMode.MODE_CLOSED
        elif mqttEvent == 4 or mqttEvent == 14:
            self.door_mode = DoorMode.MODE_OPEN


    def on_connect_mqtt(self, client, userdata, flags, rc, properties):
        if rc==0:
            self.get_logger().info(f"properties = {properties}")
            self.get_logger().info("Connected to MQTT Broker")
        else:
            self.get_logger().error(f"Unable to connect to MQTT Broker. Returned Code = {rc}")

    def on_disconnect(self, client, userdata, reasonCode, properties):
        self.get_logger().error(f"Disconnected: {reasonCode}")


    def door_open_command_request(self):
        while self.open_door:
            success = self.api.open_door()
            if success:
                self.get_logger().info(f"Request to open door [{self.door_name}] is successful")
            else:
                self.get_logger().warning(f"Request to open door [{self.door_name}] is unsuccessful")
            time.sleep(self.door_signal_period)


    def time_cb(self):
        assert self.api.sys_ping(), "Unable to ping connection with door"
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()

        # publish states of the door
        state_msg.door_name = self.door_name
        state_msg.current_mode.value = self.door_mode
        self.door_states_pub.publish(state_msg)


    def door_request_cb(self, msg: DoorRequest):
        # when door node receive open request, the door adapter will send open command to API
        # If door node receive close request, the door adapter will stop sending open command to API
        # check DoorRequest msg whether the door name of the request is same as the current door. If not, ignore the request

        self.get_logger().warn(f"DOOR REQUEST TRIGGERED... {msg}")
        # return

        if msg.door_name == self.door_name:
            self.get_logger().info(f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}")
            if msg.requested_mode.value == DoorMode.MODE_OPEN:
                # open door implementation
                self.open_door = True
                self.check_status = True
                if self.door_close_feature:
                    self.api.open_door()
                else:
                    t = threading.Thread(target = self.door_open_command_request)
                    t.start()
            elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
                # close door implementation
                self.open_door = False
                self.get_logger().info('Close Command to door received')
                if self.door_close_feature:
                    self.api.close_door()
            else:
                self.get_logger().error('Invalid door mode requested. Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter",
        description="Configure and spin up door adapter for door ")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
