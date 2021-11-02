def scale_to_rgb(val, max_value=255):
    if val <= 0.25:
        r = 0
        g = int(val / 0.25 * max_value)
        b = int(max_value)
    elif val <= 0.5:
        r = 0
        g = int(max_value)
        b = int(max_value * (1.0 - (val - 0.25) / 0.25))
    elif val <= 0.75:
        r = int(max_value * (val - 0.5) / 0.25)
        g = int(max_value)
        b = 0
    else:
        r = int(max_value)
        g = int(max_value * (1.0 - (val - 0.75) / 0.25))
        b = 0
    return r, g, b


def prx_to_rgb(prx, min_prx=500, max_prx=2000, max_rgb=255):
    scale = min(max_prx - min_prx, prx - min_prx) / float(max_prx - min_prx)
    scale = min(1.0, max(scale, 0.0))
    return scale_to_rgb(scale, max_value=max_rgb)
