# path plan in python

def array_to_bool(occupancy, length):
    threshold = 30
    x = 0
    bool_array = []
    for x in range(0, length):
        if occupancy[x] < threshold:
            bool_array.append(False)
        else:
            bool_array.append(True)
        x += 1
    print bool_array
    return bool_array

def path_location (bool_array, length):
    i = 0
    count = 0
    result = 0
    location = [0, 0]
    for i in range(0, length):
        if bool_array[i] == False:
            count = 0
        else:
            count += 1
            if result < count:
                result = count
                location[0] = i-result+1
                location[1] = i
        i += 1
    print location
    return location


length = 8
test = [70, 60, 50, 10, 70, 60, 50, 40]
path_location(array_to_bool(test, length), length)
