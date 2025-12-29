# Copyright 2025 MEGAZO Technologies Pte Ltd, All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from urllib.error import HTTPError

import requests


class DoorClientAPI:

    def __init__(
            self,
            url: str,
            username: str,
            password: str,
            project_id: int,
            devices,
            logger):

        self._token = ''
        self._timeout = 5.0
        self._logger = logger
        self._prefix = url
        self._user = username
        self._passcode = password
        self._projectID = project_id
        self._devices = devices

        count = 0
        self.connected = True
        while not self._check_connection():
            if count >= 5:
                self._logger.error('Unable to connect to door client '
                                   'API after 5 attempts. Exiting...')
                self.connected = False
                break
            else:
                self._logger.warn('Unable to connect to door client '
                                  f'API. Attempting to reconnect [{str(count)}/5]...')
                count += 1
            time.sleep(1)

    def _check_connection(self) -> bool:
        if self._login():
            if self._signin_project():
                if self.check_devices_online():
                    return True
        return False

    def _login(self) -> bool:
        """Return True if authentication token has been retrieved successfully."""
        url = self._prefix + '/API/System/Login'
        data = {}
        data['data'] = {'UserID': self._user, 'Password': self._passcode}

        try:
            response = requests.post(url, timeout=self._timeout, json=data)
            data = response.json()

            if data['IsSuccess'] is True:
                self._token = data['data']['Token']
                return True
            else:
                self._logger.error(f'RAW - data: {data}')
                self._logger.error('API:Login Failed')

        except HTTPError as http_err:
            self._logger.error(f'HTTP error during Login: {http_err}')
        except Exception as err:
            self._logger.error(f'Other error during Login: {err}')
        return False

    def _signin_project(self) -> bool:
        """Return True if specific project has been successfully signed into."""
        url = self._prefix + '/API/System/ProjectSignIn'
        data = {}
        data['UserID'] = self._user
        data['Token'] = self._token
        data['data'] = {'ProjectID': self._projectID}
        self._logger.debug(f'payload = {data}')
        try:
            response = requests.post(url, timeout=self._timeout, json=data)
            data = response.json()

            if data['IsSuccess'] is True:
                return True
            else:
                self._logger.error(f'data = {data}')
                self._logger.error('API:Project Sign-in Failed')

        except HTTPError as http_err:
            self._logger.error(f'HTTP error during Project Sign-in: {http_err}')
        except Exception as err:
            self._logger.error(f'Other error during Project Sign-in: {err}')
        return False

    def check_devices_online(self) -> bool:
        """Return True if all devices in project is online."""
        url = self._prefix + '/API/ICED/GetICEDList'
        data = {}
        data['UserID'] = self._user
        data['Token'] = self._token

        try:
            response = requests.post(url, timeout=self._timeout, json=data)
            data = response.json()

            if data['IsSuccess'] is True:
                for i in range(len(data['data'])):
                    for _, device_data in self._devices.items():
                        if data['data'][i]['ID'] == device_data['ICED_id']:
                            if data['data'][i]['ControllerStatus'] == 0:
                                self._logger.error('API: Device '
                                                   '[ {data["data"][i]["ICEDName"]} ] - '
                                                   '[ OFFLINE ]')
                                return False
                            break
        except HTTPError as http_err:
            self._logger.error(f'HTTP error during check_devices_online: {http_err}')
        except Exception as err:
            self._logger.error(f'Other error during check_devices_online: {err}')

        return True

    def command_door(self, device_ICED_id: int, op_action: int) -> bool:
        """Return True if the door API server returns successful when door command is received."""
        url = self._prefix + '/API/Device/ICED/ControlDoor'
        data = {}
        data['UserID'] = self._user
        data['Token'] = self._token
        data['data'] = {'IDs': [device_ICED_id], 'Operate': op_action}

        try:
            response = requests.post(url, timeout=self._timeout, json=data)
            data = response.json()

            if data['IsSuccess'] is True:
                return True
            else:
                self._logger.error('API: Open Door Failed')

        except HTTPError as http_err:
            self._logger.error(f'HTTP error during Open Door: {http_err}')
        except Exception as err:
            self._logger.error(f'Other error during Open Door: {err}')
        return False

    def ping_system(self) -> bool:
        """Return True if ICADR system was pinged for token refreshing."""
        url = self._prefix + '/API/SYSTEM/Ping'
        data = {}
        data['UserID'] = self._user
        data['Token'] = self._token

        try:
            response = requests.post(url, timeout=self._timeout, json=data)
            data = response.json()

            if data['IsSuccess'] is True:
                return True
            else:
                self._logger.error('API: System Ping Failed')

        except HTTPError as http_err:
            self._logger.error(f'HTTP error during System Ping: {http_err}')
        except Exception as err:
            self._logger.error(f'Other error during System Ping: {err}')
        return False
