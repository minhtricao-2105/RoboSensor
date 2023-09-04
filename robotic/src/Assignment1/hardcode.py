import math

def brick_locations():
    # Setup the location of the brick:
    brick_1_location = [20.42, -92.01, -80.04, -97.19, 89.25, 21.69]
    brick_2_location = [-3.2, -102.99, -66.44, -100.69, 89.94,  -4.45]
    brick_3_location = [-19.19, -113.95, -52.55, -102.11, 89.27, -16.07]
    brick_4_location = [18.88, -84.02, -87.02, -99.05, 90.04, 0]

    # Convert to radian:
    brick_1_location = [math.radians(angle) for angle in brick_1_location]
    brick_2_location = [math.radians(angle) for angle in brick_2_location]
    brick_3_location = [math.radians(angle) for angle in brick_3_location]
    brick_4_location = [math.radians(angle) for angle in brick_4_location]

    # Append the brick locations to the brick_locations list
    brick_locations = []
    brick_locations.append(brick_1_location)
    brick_locations.append(brick_2_location)
    brick_locations.append(brick_3_location)
    brick_locations.append(brick_4_location)

    return brick_locations

def brick_dropping():
    # Brick Dropping Positions:
    brick_1_dropping = [-94.48, -86.36, -73.41, -109.51, 89.18, 0]
    brick_2_dropping = [-94.48, -86.36, -73.41, -109.51, 89.18, 0]
    brick_3_dropping = [-94.48, -86.36, -73.41, -109.51, 89.18, 0]
    brick_4_dropping = [-45.63, -86.36, -73.41, -109.51, 89.18, 0]

    # Convert to radian:
    brick_1_dropping = [math.radians(angle) for angle in brick_1_dropping]
    brick_2_dropping = [math.radians(angle) for angle in brick_2_dropping]
    brick_3_dropping = [math.radians(angle) for angle in brick_3_dropping]
    brick_4_dropping = [math.radians(angle) for angle in brick_4_dropping]

    # Append the brick dropping to the brick_droppings list
    brick_droppings = []
    brick_droppings.append(brick_1_dropping)
    brick_droppings.append(brick_2_dropping)
    brick_droppings.append(brick_3_dropping)
    brick_droppings.append(brick_4_dropping)
    
    return brick_droppings