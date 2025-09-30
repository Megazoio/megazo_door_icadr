import sys
import yaml
import argparse

import time
import threading

import rclpy
from megazo_door_icadr.DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode

import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes 
import json


class Door:
    def __init__(self,
                 id,
                 door_auto_closes,
                 door_signal_period,
                 continuous_status_polling,
                 iced_id):
        self.id = id
        self.door_mode = DoorMode.MODE_CLOSED
        self.open_door = False
        self.check_status = None  # set to None if not enabled
        self.door_auto_closes = door_auto_closes
        self.door_signal_period = door_signal_period
        self.iced_id = iced_id
        if continuous_status_polling:
            self.check_status = False

###############################################################################

class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__('door_adapter')
        self.get_logger().info('Starting door adapter...')

        # Get value from config file
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']

        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']
        self.mock_adapter = config_yaml.get('mock', False)

        # Connect to doors
        if not self.mock_adapter:
            self.api = DoorClientAPI(self, config_yaml)

            assert self.api.connected, "Unable to establish connection with door"

            # Keep track of doors
            self.doors = {}
            for door_id, door_data in config_yaml['doors'].items():
                # We support both door_auto_closes and the deprecated
                # door_close_feature for backward compatibility
                auto_close = door_data.get('door_auto_closes', None)
                if auto_close is None:
                    if 'door_close_feature' in door_data:
                        auto_close = not door_data['door_close_feature']
                assert auto_close is not None

                iced_id = self.api.check_device_online(door_id)

                assert iced_id is not None, "Megazo IoT is not online for [{door_id}]"
                self.get_logger().info(f"Megazo device ID [{iced_id}] for [{door_id}] is online")

                self.doors[door_id] = Door(door_id,
                                           auto_close,
                                           door_data['door_signal_period'],
                                           door_data.get('continuous_status_polling', False),
                                           iced_id)
                
        self.door_states_pub = self.create_publisher(
            DoorState, door_pub['topic_name'], 100)

        self.door_request_sub = self.create_subscription(
            DoorRequest, door_sub['topic_name'], self.door_request_cb, 100)

        self.periodic_timer = self.create_timer(
            self.door_state_publish_period, self.time_cb)

        #MQTT

        # Get value from config file
        mqtt_endpoint = config_yaml['mqtt_endpoint']
        mqtt_port = config_yaml['mqtt_port']
        mqtt_topic = config_yaml['mqtt_topic']
        username = config_yaml['username']
        mqtt_password = config_yaml['mqtt_password']
        mqtt_client_id = config_yaml['mqtt_client_id']
        project_id = config_yaml['project_id']

        # Create the MQTT client instance
        client = mqtt.Client(client_id=mqtt_client_id, protocol=mqtt.MQTTv5, transport='websockets')
        client.username_pw_set(username,mqtt_password)
        
        # Set the callback functions
        client.on_connect=self.on_connect_mqtt
        client.message_callback_add(mqtt_topic, self.on_message_mqtt)

        try:
            # Define user properties
            properties=Properties(PacketTypes.PUBLISH)
            properties.UserProperty=[("ProjectID",str(project_id))]

            # Connect to the broker via WebSocket with the user properties (MQTT v5)
            client.tls_set()
            client.connect(mqtt_endpoint, mqtt_port, 60, properties=properties)

        except:
            self.get_logger().error('MQTT connection fail')
            exit(1) #Should quit or raise flag to quit or retry

        #subscribe
        client.subscribe(mqtt_topic, qos=0)

        client.loop_start() #start the loop

        #publish initial door state
        for door_id, door_data in self.doors.items():

            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()

            # publish states of the door
            state_msg.door_name = door_id
            state_msg.current_mode.value = door_data.door_mode
            self.door_states_pub.publish(state_msg)

            self.get_logger().info(f"Initializing door states for [{door_id}]")
            

    def on_message_mqtt(self, client, userdata, msg):
        #print(msg.payload)
        decoded_message=str(msg.payload.decode("utf-8"))
        msg_=json.loads(decoded_message)
        mqttEvent = msg_['doorEvent']['EventType']
        mqttId = msg_['doorEvent']['ControllerName']
        self.get_logger().info(f"New MQTT Event: = {mqttEvent}")
        self.get_logger().info(f"{msg_}")

        for door_id, door_data in self.doors.items():

            #Publish only the status of the doors indicated in the config file
            if door_id == mqttId:
                if mqttEvent == 3:
                    door_data.door_mode = DoorMode.MODE_CLOSED
                elif mqttEvent == 4 or mqttEvent == 14:
                    door_data.door_mode = DoorMode.MODE_OPEN

                state_msg = DoorState()
                state_msg.door_time = self.get_clock().now().to_msg()

                # publish states of the door
                state_msg.door_name = door_id
                state_msg.current_mode.value = door_data.door_mode
                self.door_states_pub.publish(state_msg)


    def on_connect_mqtt(self, client, userdata, flags, rc, properties):
        if rc==0:
            self.get_logger().info("Connected to MQTT Broker")
        else:
            self.get_logger().error(f"Unable to connect to MQTT Broker. Returned Code = {rc}")


    def door_open_command_request(self, door_data: Door):
        # assume API doesn't have close door API
        # Once the door command is posted to the door API,
        # the door will be opened and then close after 5 secs    
        while door_data.open_door:
            success = self.api.open_door(door_data.iced_id)
            if success:
                self.get_logger().info(f"Request to open door [{door_data.id}] is successful")
            else:
                self.get_logger().warning(f"Request to open door [{door_data.id}] is unsuccessful")
            time.sleep(door_data.door_signal_period)

    def time_cb(self):
        if self.mock_adapter:
            return
        assert self.api.sys_ping(), "Unable to ping connection with door"


    def door_request_cb(self, msg: DoorRequest):
        # Agree to every request automatically if this is a mock adapter
        if self.mock_adapter:
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()
            state_msg.door_name = msg.door_name
            state_msg.current_mode.value = msg.requested_mode.value
            self.door_states_pub.publish(state_msg)
            return

        # Check if this door has been stored in the door adapter. If not, ignore
        door_data = self.doors.get(msg.door_name)
        if door_data is None:
            return

        # When the adapter receives an open request, it will send an open
        # command to API. When the adapter receives a close request, it will
        # stop sending the open command to API
        self.get_logger().info(
            f"[{msg.door_name}] Door mode [{msg.requested_mode.value}] "
            f"requested by {msg.requester_id}"
        )
        if msg.requested_mode.value == DoorMode.MODE_OPEN:
            # open door implementation
            door_data.open_door = True
            if door_data.check_status is not None:
                # If check_status is enabled, we toggle it to true to allow
                # door state updates
                door_data.check_status = True
            if not door_data.door_auto_closes:
                self.get_logger().info(f'Calling [{msg.door_name}] ICED ID [{door_data.iced_id}] open door API')
                self.api.open_door(door_data.iced_id)
            else:
                t = threading.Thread(target=self.door_open_command_request,
                                     args=(door_data,))
                t.start()
        elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
            # close door implementation
            door_data.open_door = False
            self.get_logger().info(f'[{msg.door_name}] Close Command to door received')
            if not door_data.door_auto_closes:
                self.get_logger().info(f'Calling [{msg.door_name}] ICED ID [{door_data.iced_id}] close door API')
                self.api.close_door(door_data.iced_id)
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
