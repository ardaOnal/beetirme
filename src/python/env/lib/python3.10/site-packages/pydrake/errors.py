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

This file contains all types of custom Errors that this API can throw
"""


class APIError(Exception):
    """
    This will be raised whenever a problem occurs when communicating with the
    Riot API
    """
    pass


class RateLimitError(Exception):
    """
    This will be raised whenever a 429 response code is received from the API
    in `hard` rate-limit mode.
    """
    pass


class RateLimitWarning(Warning):
    """
    This will be raised whenever a 429 response code is received from the API
    in `soft` rate-limit mode.
    """
    pass
