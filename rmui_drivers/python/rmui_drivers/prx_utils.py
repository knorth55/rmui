from force_proximity_ros.msg import Proximity


def get_proximity_msg(prx_d, average, fa2, ea, sensitivity):
    prx_msg = Proximity()
    if prx_d is None:
        prx_msg.mode = "X"
    else:
        average = ea * prx_d + (1 - ea) * average
        fa2 = average - prx_d
        fa2derivative = average - prx_d - fa2

        prx_msg.proximity = prx_d
        prx_msg.average = average
        prx_msg.fa2 = fa2
        prx_msg.fa2derivative = fa2derivative
        if fa2 < - sensitivity:
            prx_msg.mode = "T"
        elif fa2 > sensitivity:
            prx_msg.mode = "R"
        else:
            prx_msg.mode = "0"
    return prx_msg, average, fa2
