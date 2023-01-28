from pydrake.common import configure_logging as configure_logging
from pydrake.visualization._meldis import Meldis as _Meldis

class Meldis(_Meldis):
    def __init__(self, **kwargs) -> None: ...
