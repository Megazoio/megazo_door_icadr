import requests
import time
from rmf_door_msgs.msg import DoorMode
from urllib.error import HTTPError
 
namespace = '{http://www.w3.org/1999/xhtml}'

class DoorClientAPI:
    def __init__(self, node, config):
        self.name = 'rmf_door_adapter'
        self.timeout = 5  # seconds
        self.debug = False
        self.connected = False
        self.node = node
        self.config = config  # use this config to establish connection

        self.prefix = self.config["api_endpoint"]
        self.user = self.config["username"]
        self.passcode = self.config["password"]
        self.projectID = self.config["project_id"]
        self.token = ''

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= self.timeout:
                print("Unable to connect to door client API.")
                self.connected = False
                break
            else:
                print("Unable to connect to door client API. Attempting to reconnect...")
                count += 1
            time.sleep(1)

    def check_connection(self):
        if self.Login():
            if self.ProjectSignIn():
                return True
        return False


    def Login(self):
        url = self.prefix + '/API/System/Login'
        data = {}
        data['data'] = {'UserID': self.user, 'Password': self.passcode}

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                self.token = data['data']['Token']
                return True
            else:
                print(f'API:Login Failed')

        except HTTPError as http_err:
            print(f'HTTP error during Login: {http_err}')
        except Exception as err:
            print(f'Other error during Login: {err}')
        return False


    def ProjectSignIn(self):
        url = self.prefix + '/API/System/ProjectSignIn'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'ProjectID': self.projectID}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                print(f'API:Project Sign-in Failed')

        except HTTPError as http_err:
            print(f'HTTP error during Project Sign-in: {http_err}')
        except Exception as err:
            print(f'Other error during Project Sign-in: {err}')
        return False


    def check_device_online(self, door_id) -> int:
    
        url = self.prefix + '/API/ICED/GetICEDList'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                for i in range(len(data['data'])):
                    if data['data'][i]['ICEDName'] == door_id:
                        if data['data'][i]['ControllerStatus'] == 1:
                            return data['data'][i]['ID']
                        break
            
            print(f'API: Device is offline')
                
        except HTTPError as http_err:
            print(f'HTTP error during check_device_online: {http_err}')
        except Exception as err:
            print(f'Other error during check_device_online: {err}')
        return None


    def open_door(self, iced_id):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [iced_id], 'Operate': 1}

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                print(f'API: Open Door Failed')

        except HTTPError as http_err:
            print(f'HTTP error during Open Door: {http_err}')
        except Exception as err:
            print(f'Other error during Open Door: {err}')
        return False


    def close_door(self, iced_id):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [iced_id], 'Operate': 2}

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                print(f'API: Close Door Failed')

        except HTTPError as http_err:
            print(f'HTTP error during Close Door: {http_err}')
        except Exception as err:
            print(f'Other error during Close Door: {err}')
        return False


    def sys_ping(self):
        url = self.prefix + '/API/SYSTEM/Ping'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                print(f'API: System Ping Failed')

        except HTTPError as http_err:
            print(f'HTTP error during System Ping: {http_err}')
        except Exception as err:
            print(f'Other error during System Ping: {err}')
        return False  

