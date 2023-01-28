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

This file contains all interactions with the league/v4 endpoints
"""
from .summonerv4 import Summoner

# Conversion tables for numerals and tiers
numerals = {"IV": 4, "III": 3, "II": 2, "I": 1}


class RankedSummoner(Summoner):
    def __init__(self, data, summoner):
        """
        Encapsulates information about a summoner and also provides methods to
        access their ranking information
        :param data: Data received from a league/v4 endpoint
        :param summoner: The old Summoner object to expand on
        """
        Summoner.__init__(self, summoner._raw, summoner.region)
        self._ranks = {x['queueType']: SummonerRanking(x) for x in data}

    def get_ranked_queue(self, queue_name):
        """
        Returns the data for a specified league queue
        :param queue_name: The name of the queue. Known queue names are:
        - RANKED_FLEX_SR
        - RANKED_FLEX_TT
        - RANKED_TFT
        - RANKED_SOLO_5x5
        This function will raise a ValueError if the specified queue is not
        found. These values are case sensitive
        :return: a SummonerRanking object containing the data from Riot
        """
        try:
            return self._ranks[queue_name]
        except KeyError:
            raise ValueError("{} does not have any data for queue: {}".format(self.name, queue_name))


class SummonerRanking:
    def __init__(self, data):
        """
        Encapsulates ranked information for a single queue type
        :param data: Data received from a league/v4 endpoint
        """
        self.queue_type = data['queueType']
        self.leagueId = data['leagueId']
        self.tier = data['tier']
        self.rank_str = data['rank']
        self.league_points = data['leaguePoints']
        self.wins = data['wins']
        self.losses = data['losses']
        self.games_played = self.wins + self.losses
        self.veteran = data['veteran']
        self.inactive = data['inactive']
        self.fresh_blood = data['freshBlood']
        self.hot_streak = data['hotStreak']
        self.rank = numerals[self.rank_str]

    def __str__(self):
        return "{} {} - {} LP".format(self.tier, self.rank_str, self.league_points)
