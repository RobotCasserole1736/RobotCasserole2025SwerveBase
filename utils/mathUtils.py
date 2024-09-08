def limit(inVal, maxMag):
    if inVal > maxMag:
        return maxMag
    elif inVal < -1.0 * maxMag:
        return -1.0 * maxMag
    else:
        return inVal
