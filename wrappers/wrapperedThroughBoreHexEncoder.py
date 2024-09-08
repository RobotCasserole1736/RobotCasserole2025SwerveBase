from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder


class WrapperedThroughBoreHexEncoder(WrapperedPulseWidthEncoder):
    """
    Wrappers a REV Through-bore Hex Shaft absolute encoder
    https://www.revrobotics.com/rev-11-1271/
    Assumes the absolute-angle signal from the encoder has
    been connected to a DIO port on the RoboRIO.
    """

    def __init__(self, port, name, mountOffsetRad=0.0, dirInverted=False):
        WrapperedPulseWidthEncoder.__init__(
            self, port, name, mountOffsetRad, dirInverted, 1e-6, 1.018e-3, 10.0
        )