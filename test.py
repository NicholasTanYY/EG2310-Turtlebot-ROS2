import math

def calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw):
    """
    Calculates the yaw the robot needs to turn to and the distance it needs to travel to move from point (x1, y1)
    to point (x2, y2) on a 2D plane, given its current yaw coordinate.
    Returns a tuple containing the new yaw and the distance as float values.
    """
    # Calculate the angle between the two points using the arctan2 function
    delta_x = x2 - x1
    delta_y = y2 - y1
    target_yaw = math.atan2(delta_y, delta_x)

    # Calculate the distance between the two points using the Pythagorean theorem
    distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

    # Calculate the difference between the current yaw and the target yaw

    print("target_yaw = ", target_yaw)
    print("current_yaw = ", current_yaw)
    yaw_difference = target_yaw - current_yaw

    # Normalize the yaw difference to between -pi and pi radians
    if yaw_difference > math.pi:
        yaw_difference -= 2 * math.pi
    elif yaw_difference < -math.pi:
        yaw_difference += 2 * math.pi

    return (round(yaw_difference, 3), round(distance, 3))

def move_to_waypoint(self, WP_num):
    x1, y1, x2, y2, current_yaw = self.odom_x, self.odom_y, self.waypoint_arr[WP_num][0], self.waypoint_arr[WP_num][1], self.waypoint_arr[WP_num][2]
    yaw_difference, distance = calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw)
    yaw_difference = yaw_difference / math.pi * 180

    print(f"To reach the point ({x2}, {y2}), the robot needs to turn {yaw_difference} degrees and travel {distance} units.")

# Example usage
x1 = 1
y1 = 2
x2 = -5
y2 = 6
current_yaw = math.pi / 6

yaw_difference, distance = calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw)
print(f"To reach the point ({x2}, {y2}), the robot needs to turn {yaw_difference} radians and travel {distance} units.")
