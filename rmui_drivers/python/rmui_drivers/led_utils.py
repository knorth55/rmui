def scale_to_rgb(val):
    if val <= 0.25:
        r = 0
        g = int(val / 0.25 * 255)
        b = 255
    elif val <= 0.5:
        r = 0
        g = 255
        b = 255 - int((val - 0.25) / 0.25 * 255)
    elif val <= 0.75:
        r = int((val - 0.5) / 0.25 * 255)
        g = 255
        b = 0
    else:
        r = 255
        g = 255 - int((val - 0.75) / 0.25 * 255)
        b = 0
    return r, g, b


def prx_to_rgb(prx, min_prx=500, max_prx=2000):
    scale = min(max_prx - min_prx, prx - min_prx) / (max_prx - min_prx)
    return scale_to_rgb(scale)
