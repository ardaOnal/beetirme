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

This is the main PyDrake class containing the core functions of the API
"""
import requests
import time
import warnings

from .summonerv4 import Summoner
from .leaguev4 import RankedSummoner
from .matchv4 import MatchList, Match
from .errors import *
from .ddragon import *

# Supported rate_limit_mode values
supported_modes = ["soft", "hard"]
# Supported regions for API requests
supported_regions = ["br1", "eun1", "euw1", "jp1", "kr", "la1", "la2", "na", "na1", "oc1", "tr1", "ru", "pbe1"]

# Base URL for the Riot Games API
BASE_URL = "https://{}.api.riotgames.com/lol/"


class PyDrake:
    """PyDrake is the main API wrapper class that will be used to call all
    Riot Games API functions

    :param api_key: Your Riot Games API Key (to retrieve this, see
        :ref:`api_keys`)
    :param rate_limit_mode: The rate-limiting mode to use. Defaults to `hard`.
        Can be set to:
        +----------+---------------------------------------------------------------+
        | ``soft`` | | PyDrake will attempt to execute a request, on receiving a   |
        |          | | 429 response code it will wait the amount of time specified |
        |          | | in the Retry-After response header                          |
        +----------+---------------------------------------------------------------+
        | ``hard`` | | PyDrake will raise a RateLimitError on receiving a 429      |
        |          | | response code                                               |
        +----------+---------------------------------------------------------------+

    :param show_rate_limit_warnings: Determines whether or not a
        `RateLimitWarning` will be shown when the program has to wait for a
        rate limit. Defaults to `False`.
    """
    def __init__(self, api_key, rate_limit_mode="hard", show_rate_limit_warnings=False):
        self.api_key = api_key
        self.show_rate_warnings = show_rate_limit_warnings
        if rate_limit_mode.lower() in supported_modes:
            self.rate_limit_mode = rate_limit_mode
        else:
            raise ValueError("{} is not a valid Rate Limit Mode".format(rate_limit_mode))

    def _call_api(self, region, extension):
        """
        Returns a JSON response object from the Riot Games API.
        This function should only be used internally.
        :param region: The region code of the API server to use. A ValueError
                    will be thrown if this is an invalid region code
        :param extension: The URL extension to use for the API request
        :return: dict (JSON response data)
        """
        if region.lower() not in supported_regions:
            raise ValueError("{} is not a supported service region".format(region.lower()))

        # This while loop allows the library to retry a request
        while True:
            try:
                r = requests.get(BASE_URL.format(region.lower()) + extension, params={"api_key": self.api_key})
            except requests.exceptions.ConnectionError as e:
                raise APIError(str(e))

            if r.status_code != 200:
                if r.status_code == 401:
                    raise APIError("401 Unauthorized: Your API key may be invalid")
                elif r.status_code == 403:
                    raise APIError("403 Forbidden: That endpoint may not exist or your API key may have expired")
                elif r.status_code == 404:
                    raise APIError("404 Not Found: {}".format(r.json()['status']['message']))
                elif r.status_code == 429:
                    if self.rate_limit_mode.lower() == "hard":
                        raise RateLimitError("429 Too Many Requests: Rate Limit exceeded")
                    elif self.rate_limit_mode.lower() == "soft":
                        retry = int(r.headers["retry-after"])
                        if self.show_rate_warnings:
                            warnings.warn("Rate Limit exceeded - waiting {} seconds".format(retry), RateLimitWarning)
                        time.sleep(retry)
                        continue
                elif r.status_code == 503:
                    raise APIError("503 Service Unavailable: failed to reach API endpoint")
            else:
                break

        ret = r.json()
        if ret is None:
            raise APIError("No data received from API")
        return ret

    @property
    def get_champion_by_id(self):
        """See :func:`pydrake.ddragon.get_champion_by_id`
        """
        return get_champion_by_id

    def get_summoner_by_name(self, name, region):
        """Retrieve statistics about a summoner which will be found by the
        given name.

        :param name: The name of the summoner
        :param region: The region code that the account belongs to
        :return: A :class:`pydrake.summonerv4.Summoner` object containing the
            parsed data from the API

        .. warning:: Summoner names are case-sensitive

        .. note:: For a full list of supported region codes, see
                :ref:`region_codes`.
        """
        response = self._call_api(region, "summoner/v4/summoners/by-name/{}".format(name))
        return Summoner(response, region)

    def get_ranked_summoner(self, summoner):
        """Retrieves the ranking information for the given summoner and returns a
        RankedSummoner object containing the new information

        :param summoner: The old summoner object to extend
        :return: A :class:`pydrake.leaguev4.RankedSummoner` object
        """
        response = self._call_api(summoner.region, "league/v4/entries/by-summoner/{}".format(summoner.id))
        return RankedSummoner(response, summoner)

    def get_summoner_matchlist(self, summoner):
        """Retrieves the matchlist (recent match history ~150 games) for the
        given summoner

        :param summoner: The summoner to retrieve match data for
        :return: A :class:`pydrake.matchv4.MatchList` object
        """
        response = self._call_api(summoner.region, "match/v4/matchlists/by-account/{}".format(summoner.account_id))
        return MatchList(response)

    def get_match_from_matchlist(self, match):
        """Retrieves the full data for a match from an item in a MatchList object

        :param match: The entry from a MatchList object to retrieve further
            data for
        :return: a :class:`pydrake.matchv4.Match` object containing all of the
            data about the match
        """
        response = self._call_api(match.platform_id, "match/v4/matches/{}".format(match.game_id))
        return Match(response)
