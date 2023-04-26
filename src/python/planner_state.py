from enum import Enum

class PlannerState(Enum):
    WAIT_FOR_OBJECTS_TO_SETTLE = 1
    PICKING_FROM_SHELF_1 = 2
    GO_HOME = 3