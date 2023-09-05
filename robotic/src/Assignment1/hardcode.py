import math

def brick_locations():
    # Setup the location of the brick:
    brick_1_location = [-19.19, -113.95, -52.55, -102.11, 89.27, -16.07]
    brick_2_location = [-3.2, -102.99, -66.44, -100.69, 89.94,  -4.45]
    brick_3_location = [20.42, -92.01, -80.04, -97.19, 89.25, 21.69]
    brick_4_location = [-87.11, -129.02, -29.02, -110.85, 89.94, 5.43]
    brick_5_location = [-110.31, -90.21, -80.49, -98.16, 90.18, 160]
    brick_6_location = [-106.01, -114.50, -51.14, -103.23, 90.15, 160]
    brick_7_location = [-153.41, -93.57, -77.25, -99.22, 90.0, 30]
    brick_8_location = [-135.24, -104.26, -66.85, -98.85, 90.0, 45]
    brick_9_location = [-123.33, -128.73, -26.22, -114.12, 90.0, 60]

    # Convert to radian:
    brick_1_location = [math.radians(angle) for angle in brick_1_location]
    brick_2_location = [math.radians(angle) for angle in brick_2_location]
    brick_3_location = [math.radians(angle) for angle in brick_3_location]
    brick_4_location = [math.radians(angle) for angle in brick_4_location]
    brick_5_location = [math.radians(angle) for angle in brick_5_location]
    brick_6_location = [math.radians(angle) for angle in brick_6_location]
    brick_7_location = [math.radians(angle) for angle in brick_7_location]
    brick_8_location = [math.radians(angle) for angle in brick_8_location]
    brick_9_location = [math.radians(angle) for angle in brick_9_location]

    # Append the brick locations to the brick_locations list
    brick_locations = []
    brick_locations.append(brick_1_location)
    brick_locations.append(brick_2_location)
    brick_locations.append(brick_3_location)
    brick_locations.append(brick_4_location)
    brick_locations.append(brick_5_location)
    brick_locations.append(brick_6_location)
    brick_locations.append(brick_7_location)
    brick_locations.append(brick_8_location)
    brick_locations.append(brick_9_location)
    
    return brick_locations
    

def brick_dropping():
    # Brick Dropping Positions:
    brick_1_dropping = [-45.63, -86.36, -73.41, -109.51, 89.18, 45]
    brick_2_dropping = [-45.63, -86.36, -73.41, -109.51, 89.18, 45]
    brick_3_dropping = [-45.63, -90.76, -57.53, -120.99, 89.16, 45]
    brick_4_dropping = [-56.22, -79.12, -85.47, -104.75, 89.32, 120]
    brick_6_dropping = [-56.22, -83.30, -68.55, -117.49, 89.30, 120]
    brick_7_dropping = [-69.75, -80.8, -79.96, -108.43, 89.5, 110]
    brick_9_dropping = [-69.75, -85.25, -60.06, -123.15, 90.22, 110]
    # Convert to radian:
    brick_1_dropping = [math.radians(angle) for angle in brick_1_dropping]
    brick_2_dropping = [math.radians(angle) for angle in brick_2_dropping]
    brick_3_dropping = [math.radians(angle) for angle in brick_3_dropping]
    brick_4_dropping = [math.radians(angle) for angle in brick_4_dropping]
    brick_5_dropping = brick_4_dropping
    brick_6_dropping = [math.radians(angle) for angle in brick_6_dropping]
    brick_7_dropping = [math.radians(angle) for angle in brick_7_dropping]
    brick_8_dropping = brick_7_dropping
    brick_9_dropping = [math.radians(angle) for angle in brick_9_dropping]
    # Append the brick dropping to the brick_droppings list
    brick_droppings = []
    brick_droppings.append(brick_1_dropping)
    brick_droppings.append(brick_2_dropping)
    brick_droppings.append(brick_3_dropping)
    brick_droppings.append(brick_4_dropping)
    brick_droppings.append(brick_5_dropping)    
    brick_droppings.append(brick_6_dropping)
    brick_droppings.append(brick_7_dropping)
    brick_droppings.append(brick_8_dropping)
    brick_droppings.append(brick_9_dropping)

    return brick_droppings