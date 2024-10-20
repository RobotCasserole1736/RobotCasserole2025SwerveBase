def limit(inVal:float, maxMag:float)->float:
    """
    Prevents a value from getting larger than some magnititude (positive or negative).
    Returns the value if it's small enough, or the limit if it's too big.
    """
    if inVal > maxMag:
        return maxMag
    elif inVal < -1.0 * maxMag:
        return -1.0 * maxMag
    else:
        return inVal
