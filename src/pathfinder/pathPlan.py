# path plan in python
import math

def path_location (range_array):
    i = 0
    count = 0
    longest_pos = 0
    threshold = 40
    location_range = [0, 0]
    length = len(range_array)
    for i in range(0, length):
        if range_array[i] <= threshold or math.isnan(range_array[i]):
            count = 0
        else:
            count += 1
            if longest_pos < count:
                longest_pos = count
                location_range[0] = i-longest_pos+1
                location_range[1] = i
        i += 1
    center_point = (location_range[0]+location_range[1])/2.0
    #print location_range
    #print center_point
    return [center_point, length]

def rudder_pos(loc_array):
    '''
    normalized values:
    r = 30, l = -30, s = 0
    not normalized values:
    r = 120, l = 60, s = 90
    '''
    location = loc_array[0]
    length = loc_array[1]
    slope = 30/(length/2.0)
    intercept = -30.0
    position = ((slope*location) + intercept) + 90.0
    #print position
    return int(position)

length = 10
test = [70, 60, 50, 10, 70, 60, 50, 50, 50, 70]
print (rudder_pos(path_location(test)))
