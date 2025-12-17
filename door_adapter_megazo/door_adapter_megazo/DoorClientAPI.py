import requests
import time
from rmf_door_msgs.msg import DoorMode
from urllib.error import HTTPError


class DoorClientAPI:

    def __init__(
            self,
            url,
            username,
            password,
            project_id,
            devices,
            logger
        ):
        self.prefix = url
        self.timeout = 5.0
        self.user = username
        self.passcode = password
        self.projectID = project_id
        self.devices = devices
        self.token = ''
        self.logger = logger

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
                self.logger.error("Unable to connect to door client API.")
                self.connected = False
                break
            else:
                self.logger.warn("Unable to connect to door client API. Attempting to reconnect...")
                count += 1
            time.sleep(1)


    def check_connection(self):
        if self.Login():
            if self.ProjectSignIn():
                if self.check_devices_online():
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
                self.logger.error(f"RAW - data: {data}")
                self.logger.error(f'API:Login Failed')

        except HTTPError as http_err:
            self.logger.error(f'HTTP error during Login: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during Login: {err}')
        return False


    def ProjectSignIn(self):
        url = self.prefix + '/API/System/ProjectSignIn'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'ProjectID': self.projectID}
        self.logger.debug(f"payload = {data}")
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                self.logger.error(f"data = {data}")
                self.logger.error(f'API:Project Sign-in Failed')

        except HTTPError as http_err:
            self.logger.error(f'HTTP error during Project Sign-in: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during Project Sign-in: {err}')
        return False


    def check_devices_online(self):
        url = self.prefix + '/API/ICED/GetICEDList'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                for i in range(len(data['data'])):
                    for _, device_data in self.devices.items():
                        if data['data'][i]['ID'] == device_data['ICED_id']:
                            if data['data'][i]['ControllerStatus'] == 1:
                                return True
                            break

            self.logger.error(f'API: Device is offline')
                
        except HTTPError as http_err:
            self.logger.error(f'HTTP error during check_devices_online: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during check_devices_online: {err}')
        return False


    def open_door(self, device_ICED_id):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [device_ICED_id], 'Operate': 1}

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                self.logger.error(f'API: Open Door Failed')

        except HTTPError as http_err:
            self.logger.error(f'HTTP error during Open Door: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during Open Door: {err}')
        return False


    def close_door(self, device_ICED_id):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [device_ICED_id], 'Operate': 2}

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                return True
            else:
                self.logger.error(f'API: Close Door Failed')

        except HTTPError as http_err:
            self.logger.error(f'HTTP error during Close Door: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during Close Door: {err}')
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
                self.logger.error(f'API: System Ping Failed')

        except HTTPError as http_err:
            self.logger.error(f'HTTP error during System Ping: {http_err}')
        except Exception as err:
            self.logger.error(f'Other error during System Ping: {err}')
        return False 
