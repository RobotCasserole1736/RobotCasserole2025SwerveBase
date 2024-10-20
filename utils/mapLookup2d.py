from operator import itemgetter


# pylint: disable=too-few-public-methods
class MapLookup2D:
    """
    A map-lookup is a common tool to define a function. The user must provide samples of the 
    function's output value (y) at different inputs (x). This function will linearally interpolate
    between the provided points. The asusmption is the function is "piecewise linear". As long
    as enough sample points are provided, this method can well-aproximate just about any function.
    
    Points should be a list of two-element tuples, with (x,y) pairs described by each tuple.

    """
    def __init__(self, points: list[tuple[float, float]]):
        self.points = points
        # Ensure points list is ordered from lowest value to highest value
        self.points.sort(key=itemgetter(0))

    def lookup(self, xval):
        yval = 0
        if len(self.points) == 0:
            pass  # case, no points. nothing we can do.
        elif len(self.points) == 1:
            # One point. yup, that's the one.s
            yval = self._y(0)
        else:
            # multiple points. Normal algorithm applies
            xMin = self._x(0)
            xMax = self._x(-1)
            if xval < xMin:
                # off the lower end of the map, return smallest defined point
                return self._y(0)
            elif xval > xMax:
                # off the upper end of the map, return the highest defined point
                return self._y(-1)
            else:
                # Within the map
                lowerIndex = 0
                # Iterate over the xAxis array to find the interval that req_x_val falls within
                # Note this only works because xAxis is sorted.
                for indexIter in range(0, len(self.points) - 1):
                    if self._x(indexIter) <= xval <= self._x(indexIter + 1):
                        lowerIndex = indexIter
                        break

                # math math math math mathy math
                intervalXDelta = self._x(lowerIndex + 1) - self._x(lowerIndex)
                intervalYDelta = self._y(lowerIndex + 1) - self._y(lowerIndex)
                reqFractionIntoInterval = (xval - self._x(lowerIndex)) / intervalXDelta
                return (reqFractionIntoInterval * intervalYDelta) + self._y(lowerIndex)

        return yval

    # pylint: disable=invalid-name
    def _x(self, idx):
        return self.points[idx][0]

    # pylint: disable=invalid-name
    def _y(self, idx):
        return self.points[idx][1]
