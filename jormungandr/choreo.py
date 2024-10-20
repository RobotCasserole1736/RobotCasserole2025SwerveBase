import json
from jormungandr.choreoTrajectory import ChoreoTrajectoryState
from jormungandr.choreoTrajectory import ChoreoTrajectory


def fromFile(file: str) -> ChoreoTrajectory:
    """
    Handles reading a choreo trajectory file from disk,
    reading all the data into it, and creating the list of 
    trajectory states that we'll later use to control the robot
    """
    samples = []
    with open(file, "r", encoding="utf-8") as trajFile:
        data = json.load(trajFile)
        for sample in data["samples"]:
            samples.append(
                ChoreoTrajectoryState(
                    float(sample["timestamp"]),
                    float(sample["x"]),
                    float(sample["y"]),
                    float(sample["heading"]),
                    float(sample["velocityX"]),
                    float(sample["velocityY"]),
                    float(sample["angularVelocity"]),
                )
            )

    return ChoreoTrajectory(samples)
