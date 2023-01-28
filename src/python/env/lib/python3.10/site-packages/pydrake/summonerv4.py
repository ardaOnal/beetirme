"""
   Copyright 2019 Jacob Padley

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

This file contains all interactions with the summoner/v4 endpoints
"""


class Summoner:
    def __init__(self, data, region):
        """
        Contains information received from the summoner/v4 endpoints
        :param data: The data received from the endpoint as a dict
        :param region: The region code that this summoner is associated with
        """
        # Parse response data
        self.name = data['name']
        self.id = data['id']
        self.account_id = data['accountId']
        self.puuid = data['puuid']
        self.iconId = data['profileIconId']
        self.level = data['summonerLevel']
        self.region = region

        # This is only for later creating a Ranked Summoner object
        self._raw = data
