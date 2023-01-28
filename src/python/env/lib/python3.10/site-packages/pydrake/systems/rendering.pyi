import pydrake.multibody.plant
import pydrake.systems.framework

class MultibodyPositionToGeometryPose(pydrake.systems.framework.LeafSystem_[float]):
    def __init__(self, plant: pydrake.multibody.plant.MultibodyPlant_[float], input_multibody_state: bool = ...) -> None: ...
