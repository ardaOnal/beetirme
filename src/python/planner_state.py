from enum import Enum

class PlannerState(Enum):
    WAIT_FOR_OBJECTS_TO_SETTLE = 1
    GO_TO_SHELF = 2
    PICKING_FROM_SHELF_1 = 3
    GO_HOME = 4