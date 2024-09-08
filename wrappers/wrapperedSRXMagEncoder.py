from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder


class WrapperedSRXMagEncoder(WrapperedPulseWidthEncoder):
    """
    Wrappers a CTRE SRX Magnetic absolute encoder
    https://store.ctr-electronics.com/srx-mag-encoder/
    Assumes the absolute-angle signal from the encoder has
    been connected to a DIO port on the RoboRIO.
    """

    def __init__(self, port, name, mountOffsetRad, dirInverted):
        WrapperedPulseWidthEncoder.__init__(
            self, port, name, mountOffsetRad, dirInverted, 1e-6, 4.096e-3, 10.0
        )