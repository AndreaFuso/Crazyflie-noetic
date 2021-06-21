def constrain(value, lowerVal, upperVal):
    if value < lowerVal:
        return lowerVal
    if value > upperVal:
        return upperVal
    return value