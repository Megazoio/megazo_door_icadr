import sys
import yaml
import json
import time
import argparse
import threading
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes 

import rclpy
from rclpy.node import Node
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode

from door_adapter_megazo.DoorClientAPI import DoorClientAPI
from door_adapter_megazo.utils.Constants import NODE_NAME, MQTT_EVENTTYPE


class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__(NODE_NAME)
        self.get_logger().info(f'Initialising [ {NODE_NAME} ]...')

        # Get value from config file
        self.get_logger().debug(f'Adding {len(config_yaml['doors'])} doors...')

        self.doors = {}
        for key, value in config_yaml['doors'].items():
            self.get_logger().debug(f'Adding door: [ {key} ]')
            self.get_logger().debug(f'Door Data = [ {value} ]')
            self.doors[key] = value
            # Default door mode is CLOSED.
            self.doors[key]['door_mode'] = DoorMode.MODE_CLOSED

        url = config_yaml['api_endpoint']
        mqtt_endpoint = config_yaml['mqtt_endpoint']
        mqtt_port = config_yaml['mqtt_port']

        mqtt_topic = config_yaml['mqtt_topic']
        username = config_yaml['username']
        password = config_yaml['password']
        mqtt_password = config_yaml['mqtt_password']
        mqtt_client_id = config_yaml['mqtt_client_id']
        project_id = config_yaml['project_id']

        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']

        self.api = DoorClientAPI(
            url,
            username,
            password,
            project_id,
            self.doors,
            self.get_logger()
        )

        assert self.api.connected, "Unable to establish connection with door"

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
            self.get_logger().error('MQTT Connection - [ FAILED ]')
            exit(1) #Should quit or raise flag to quit or retry

        #subscribe
        client.subscribe(mqtt_topic, qos=0)

        client.loop_start() #start the loop

        self.get_logger().info(f'[ {NODE_NAME} ] - [ RUNNING ]...')

    def on_message_mqtt(self, client, userdata, msg):
        #print(msg.payload)
        decoded_message=str(msg.payload.decode("utf-8"))
        msg_=json.loads(decoded_message)
        mqttEvent = msg_['doorEvent']['EventType']

        detected_device_name = None
        for device_name, _ in self.doors.items():
            if device_name in msg_['doorEvent']['ControllerName']:
                detected_device_name = device_name

        if detected_device_name is None:
            self.get_logger().debug(f"No devices matching [ {msg_['doorEvent']['ControllerName']} ]. Ignoring...")
            return

        self.get_logger().info(f"Detected New MQTT Event - [{mqttEvent}][ {MQTT_EVENTTYPE[str(mqttEvent)]} ] on [ {msg_['doorEvent']['ControllerName']} ]")
        self.get_logger().debug(f"{msg_}")

        if mqttEvent == 1020:
            self.doors[detected_device_name]['door_mode'] = DoorMode.MODE_CLOSED
        elif mqttEvent == 1019:
            self.doors[detected_device_name]['door_mode'] = DoorMode.MODE_OPEN


    def on_connect_mqtt(self, client, userdata, flags, rc, properties):
        if rc==0:
            self.get_logger().warn("Connection to MQTT Broker - [ SUCCESS ]")
        else:
            self.get_logger().error(f"Unable to connect to MQTT Broker. Returned Code = {rc}")

    def on_disconnect(self, client, userdata, reasonCode, properties):
        self.get_logger().error(f"Disconnected: {reasonCode}")


    def door_open_command_request(self, door_name: str):
        while self.open_door:
            success = self.api.open_door(self.doors[door_name]['ICED_id'])
            if success:
                self.get_logger().warn(f"Request to open door [{door_name}] is successful")
            else:
                self.get_logger().warn(f"Request to open door [{door_name}] is unsuccessful")
            time.sleep(self.doors[door_name]['door_signal_period'])


    def time_cb(self):
        assert self.api.sys_ping(), "Unable to ping connection with door"
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()

        # Publish states of the door
        for door_name, _ in self.doors.items():
            state_msg.door_name = door_name
            state_msg.current_mode.value = self.doors[door_name]['door_mode']
            self.door_states_pub.publish(state_msg)


    def door_request_cb(self, msg: DoorRequest):
        # when door node receive open request, the door adapter will send open command to API
        # If door node receive close request, the door adapter will stop sending open command to API
        # check DoorRequest msg whether the door name of the request is same as the current door. If not, ignore the request

        self.get_logger().warn(f"Door Request - [ RECEIVED ]")
        self.get_logger().debug(f"Door Request - {msg}")

        is_requested_door_in_list = False
        for door_name, _ in self.doors.items():
            if door_name == msg.door_name:
                is_requested_door_in_list = True
                break

        if is_requested_door_in_list:
            self.get_logger().info(f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}")
            if msg.requested_mode.value == DoorMode.MODE_OPEN:
                # open door implementation
                self.open_door = True
                self.check_status = True
                if self.doors[msg.door_name]['door_close_feature']:
                    self.get_logger().info('Commanding Door - [ OPEN ]')
                    self.api.open_door(self.doors[msg.door_name]['ICED_id'])
                else:
                    t = threading.Thread(target = self.door_open_command_request(msg.door_name))
                    t.start()
            elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
                # close door implementation
                self.open_door = False
                if self.doors[msg.door_name]['door_close_feature']:
                    self.get_logger().info('Commanding Door - [ CLOSE ]')
                    self.api.close_door(self.doors[msg.door_name]['ICED_id'])
                else:
                    self.get_logger().info(f'Door [ {msg.door_name} ] does not have Close Door API. Ignoring...')
            else:
                self.get_logger().error('Invalid door mode requested. Ignoring...')
        else:
            self.get_logger().error(f'Unable to find requested door [ {msg.door_name} ]. Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog=NODE_NAME,
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
