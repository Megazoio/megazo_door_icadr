import requests
import time
from rmf_door_msgs.msg import DoorMode
from urllib.error import HTTPError
 
namespace = '{http://www.w3.org/1999/xhtml}'

class DoorClientAPI:

    def __init__(self,url,username,password,project_id,ICED_id):
        self.prefix = url
        self.timeout = 5.0
        self.user = username
        self.passcode = password
        self.projectID = project_id
        self.ID = ICED_id
        self.token = ''

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
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
                if self.check_device_online():
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


    def check_device_online(self):
        url = self.prefix + '/API/ICED/GetICEDList'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                for i in range(len(data['data'])):
                    if data['data'][i]['ID'] == self.ID:
                        if data['data'][i]['ControllerStatus'] == 1:
                            return True
                        break
            
            print(f'API: Device is offline')
                
        except HTTPError as http_err:
            print(f'HTTP error during check_device_online: {http_err}')
        except Exception as err:
            print(f'Other error during check_device_online: {err}')
        return False


    def open_door(self):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [self.ID], 'Operate': 1}

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


    def close_door(self):
        ''' Return True if the door API server is successful receive open door command'''
        url = self.prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token
        data['data'] = {'IDs': [self.ID], 'Operate': 2}

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


    # NOT IN USE. USING MQTT TO GET STATUS.
    def get_mode(self):        
        ''' Return the door status with reference rmf_door_msgs. 
            Return DoorMode.MODE_CLOSED when door status is closed.
            Return DoorMode.MODE_MOVING when door status is moving.
            Return DoorMode.MODE_OPEN when door status is open.
            Return DoorMode.MODE_OFFLINE when door status is offline.
            Return DoorMode.MODE_UNKNOWN when door status is unknown'''
        url = self.prefix + '/API/ICED/GetICEDList'
        data = {}
        data['UserID'] = self.user
        data['Token'] = self.token

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            data = response.json()

            if data['IsSuccess'] == True:
                for i in range(len(data['data'])):
                    if data['data'][i]['ID'] == self.ID:
                        if data['data'][i]['DoorOpenStatus'] == 1:
                            return DoorMode.MODE_OPEN
                        else:
                            return DoorMode.MODE_CLOSED
            else:
                print(f'API: Door get_mode Failed')

        except HTTPError as http_err:
            print(f'HTTP error during Door get_mode: {http_err}')
        except Exception as err:
            print(f'Other error during Door get_mode: {err}')
        return DoorMode.MODE_UNKNOWN


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

        

