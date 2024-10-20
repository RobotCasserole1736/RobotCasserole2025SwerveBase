from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from utils.singleton import Singleton
from wpimath.geometry import Pose3d


class FieldTagLayout(metaclass=Singleton):
    """
    Very thin wrapper around WPILib's apriltag field layout functions. But should expand in the future
    """
    def __init__(self):
        """
        Performs a one-time load from file of the apriltag field layout .json
        """

        # TODO - this actually goes and accesses files under the hood. And... maybe...
        # Throws exceptions if it has issues accessing the file. We should be catching
        # those exceptions so the robot code doesn't outright crash, but raise a fault
        # to indicate something has gone wrong with the RIO's ability to load the file.
        self.fieldTags = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

    def lookup(self, tagId) -> Pose3d | None:
        """
        Returns a Pose3d of a given tag's location in 3d space. Or, None if the id
        isn't a tag we know a location for.
        """
        # TODO - if the file couldn't be loaded successfully, this needs to return None
        return self.fieldTags.getTagPose(tagId)
