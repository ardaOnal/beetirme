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
"""
from .errors import APIError


class Match:
    def __init__(self, data):
        """
        Encapsulates data retrieved from a match/v4/matches call
        :param data: the data to parse
        """
        self.game_id = data['gameId']
        self.platform_id = data['platformId']
        self.game_creation = data['gameCreation']
        self.game_duration = data['gameDuration']
        self.queue_id = data['queueId']
        self.map_id = data['mapId']
        self.season_id = data['seasonId']
        self.game_version = data['gameVersion']
        self.game_mode = data['gameMode']
        self.game_type = data['gameType']
        self.teams = [Team(x) for x in data['teams']]
        self.participants = self._parse_participants(data)

    def get_participant_by_name(self, name):
        """
        Returns a MatchParticipant object with the given name. Raises a KeyError
        if the name is not found
        :param name: The name of the participant. This is case sensitive
        :return: a MatchParticipant object
        """
        for p in self.participants:
            if p.name == name:
                return p

        raise KeyError("Could not find a participant named '{}'".format(name))

    def get_team(self, id):
        """
        Returns a team object with the given ID. Raises a KeyError if the team
        is not found
        :param id: The team's ID
        :return: A Team object
        """
        for team in self.teams:
            if team.team_id == id:
                return team

        raise KeyError("Could not find a team with ID: {}".format(id))

    def _parse_participants(self, data):
        """
        Parses the given data object to locate "participants" and
        "participantIdentities". Combines the two into MatchParticipant
        objects
        :param data: The full JSON object used to construct this object
        :return: A list of MatchParticipant objects
        """
        return [MatchParticipant(x, self._match_pair(x, data["participantIdentities"])) for x in data["participants"]]

    def _match_pair(self, participant, participant_identities):
        """
        Matches a "participant" JSON object to its corresponding
        "participantIdentity" object.
        :param participant: the original JSON participant object
        :param participant_identities: the whole participantIdentities JSON object
        :return: the matching participant_identities record
        """
        pid = participant["participantId"]
        pair = next((x for x in participant_identities if x['participantId'] == pid), None)
        if pair is None:
            raise APIError("Internal Error - no matching data found for match participant {}".format(pid))
        else:
            return pair


class MatchParticipant:
    def __init__(self, participant, participant_id):
        """
        Encapsulates a match participant found in a match/v4/matches call
        :param participant: an entry from the "participants" section of the call
        :param participant_id: the matching entry from the "participantIdentities" section
        """
        self.id = participant['participantId']
        self.team_id = participant['teamId']
        self.champion_id = participant['championId']
        self.summoner_spells = [participant['spell1Id'], participant['spell2Id']]
        self.stats = participant['stats']
        self.role = participant['timeline']['role']
        self.lane = participant['timeline']['lane']

        self.platform_id = participant_id['player']['platformId']
        self.account_id = participant_id['player']['accountId']
        self.name = participant_id['player']['summonerName']
        self.summoner_id = participant_id['player']['summonerId']
        self.profile_icon = participant_id['player']['profileIcon']


class Team:
    def __init__(self, data):
        """
        Encapsulates a team found in a match/v4/matches call
        :param data: The data retrieved
        """
        self.team_id = data['teamId']
        self.win = data['win'] == "Win"
        self.first_blood = data['firstBlood']
        self.first_tower = data['firstTower']
        self.first_inhibitor = data['firstInhibitor']
        self.first_baron = data['firstBaron']
        self.first_dragon = data['firstDragon']
        self.first_rift_herald = data['firstRiftHerald']
        self.tower_kills = data['towerKills']
        self.inhibitor_kills = data['inhibitorKills']
        self.baron_kills = data['baronKills']
        self.dragon_kills = data['dragonKills']
        self.vilemaw_kills= data['vilemawKills']
        self.rift_herald_kills = data['riftHeraldKills']
        self.dominion_victory_score = data['dominionVictoryScore']
        # We won't unpack this into an Object because there is so little data
        self.bans = data['bans']


class MatchListMatch:
    def __init__(self, data):
        """
        Encapsulates data retrieved from a match/v4/matchlists call
        :param data: The data to parse
        """
        self.platform_id = data['platformId']
        self.game_id = data['gameId']
        self.champion = data['champion']
        self.queue = data['queue']
        self.season = data['season']
        self.timestamp = data['timestamp']
        self.role = data['role']
        self.lane = data['lane']


class MatchList:
    def __init__(self, data):
        """
        Encapsulates data retrieved from a match/v4/matchlists call in the form
        of MatchListMatch objects
        :param data: the data returned from the API arranged in descending date
        order
        """
        self._matches = sorted([MatchListMatch(x) for x in data['matches']], key=lambda x: x.timestamp, reverse=True)

    def __getitem__(self, index):
        return self._matches[index]

    def __iter__(self):
        return iter(self._matches)
