from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from utils.singleton import Singleton


class FieldTagLayout(metaclass=Singleton):
    def __init__(self):
        self.fieldTags = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

    def lookup(self, tagId):
        return self.fieldTags.getTagPose(tagId)
