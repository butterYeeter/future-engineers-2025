def rgb_to_hsv(color):
    r, g, b = [x/255.0 for x in color]

    max_val = max(r, g, b)
    min_val = min(r, g, b)
    diff = max_val - min_val
    
    v = max_val * 100
    
    s = 0.0 if max_val == 0 else (diff / max_val) * 100
    
    if diff == 0:
        h = 0
    else:
        if max_val == r:
            h = 60 * (((g - b) / diff) % 6)
        elif max_val == g:
            h = 60 * (((b - r) / diff) + 2)
        else:  # max_val == b
            h = 60 * (((r - g) / diff) + 4)
    
    return (round(h, 2), round(s, 2), round(v, 2))

def is_blue(rgb_tuple, threshold=None):
    # Default blue threshold (H:200-260°, S:40-100%, V:40-100%)
    default_threshold = (200, 260, 40, 100, 40, 100)
    h_min, h_max, s_min, s_max, v_min, v_max = threshold or default_threshold
    
    h, s, v = rgb_to_hsv(rgb_tuple)
    
    # Check if within all thresholds
    return (h_min <= h <= h_max and 
            s_min <= s <= s_max and 
            v_min <= v <= v_max)