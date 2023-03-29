    rgb = rightRGB[0] * (255 / 100), rightRGB[1] * (255 / 100), rightRGB[2] * (255 / 100)
    maxRBG = max(rgb)
    minRGB = min(rgb)

    # Hue
    if maxRGB == rgb[0]: # Red 
        hue = 60 * ((rgb[1] - rgb[2]) / (maxRGB - minRGB))
    elif maxRBG == rgb[1]: # Green
        hue = 60 * ((rgb[2] - rgb[0]) / (maxRGB - minRGB)) + 120
    else: # Blue
        hue = 60 * ((rgb[0] - rgb[1]) / (maxRGB - minRGB)) + 240

    if hue < 0:
        hue += 360

    # Saturation
    saturation = (maxRGB - minRGB) / maxRGB * 100

    # Value
    value_brightness = max(rightRGB)