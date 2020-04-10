#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from drive_msgs.msg import gazebo_state_telemetry

from rviz_geometry import show_circle_in_rviz, show_line_in_rviz, delete_marker

from circle import Circle, Point

import math

import numpy as np

import time

from dynamic_reconfigure.server import Server
from wallfollowing3.cfg import wallfollowing3Config

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param"
TOPIC_LASER_SCAN = "/scan"

CAR_ACCELERATION = rospy.get_param("/physical_properties/acceleration")
CAR_DECCELERATION = CAR_ACCELERATION

FRICTION = rospy.get_param("/physical_properties/dynamic_friction")


MAX_STEERING_ANGLE = 30 * (math.pi / 180)
CAR_LENGTH = 0.325
CAR_WIDTH = 0.3

CURVE_TYPE_RIGHT = 0
CURVE_TYPE_LEFT = 1

last_angle_moving_average = [[], 8]
last_speed = 0
current_speed = 0

total_scan_time = 0
total_scan_count = 0


class Parameters():
    def __init__(self, default_values):
        self.names = default_values.keys()
        for name in self.names:
            setattr(self, name, default_values[name])

    def __str__(self):
        return '\n'.join(name + ": " + str(getattr(self, name))
                         for name in self.names)


class PIDController():
    def __init__(self, p, i, d, anti_windup=0.2):
        self.p = p
        self.i = i
        self.d = d
        self.anti_windup = anti_windup

        self.integral = 0
        self.previous_error = 0

    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        if self.integral > self.anti_windup:
            self.integral = self.anti_windup
        elif self.integral < -self.anti_windup:
            self.integral = -self.anti_windup

        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative


def map(in_lower, in_upper, out_lower, out_upper, value):
    result = out_lower + (out_upper - out_lower) * \
        (value - in_lower) / (in_upper - in_lower)
    return min(out_upper, max(out_lower, result))


def convertRpmToSpeed(rpm):
    # 1299.224 is the conversion factor from electrical revolutions per minute to m/s
    # and is derived from the transmission and the rotational speed of the motor.
    # More values describing the car properties can be found in car_config.h.
    return rpm / 1299.224


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def get_scans_in_angular_range(laser_scan, angle_start, angle_end):
    ranges = np.array(laser_scan.ranges)
    filterd_ranges = ranges[int((laser_scan.angle_max + angle_start) / laser_scan.angle_increment): int((laser_scan.angle_max + angle_end) / laser_scan.angle_increment)]
    inf_mask = np.isinf(filterd_ranges)
    if inf_mask.any():
        filterd_ranges = filterd_ranges[~inf_mask]
    return filterd_ranges


def get_scan_as_cartesian(laser_scan):
    ranges = np.array(laser_scan.ranges)

    angles = np.linspace(
        laser_scan.angle_min,
        laser_scan.angle_max,
        ranges.shape[0])

    laser_range = laser_scan.angle_max - laser_scan.angle_min
    usable_range = math.radians(parameters.usable_laser_range)
    if usable_range < laser_range:
        skip_left = int((-laser_scan.angle_min - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        skip_right = int((laser_scan.angle_max - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        angles = angles[skip_left:-1 - skip_right]
        ranges = ranges[skip_left:-1 - skip_right]

    inf_mask = np.isinf(ranges)
    if inf_mask.any():
        ranges = ranges[~inf_mask]
        angles = angles[~inf_mask]

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    return points


def find_left_right_border(points):
    relative = points[1:, :] - points[:-1, :]
    distances = np.linalg.norm(relative, axis=1)

    split_index = np.argmax(distances)

    show_line_in_rviz(15, [Point(points[split_index][0], points[split_index][1]), Point(points[split_index + 1][0], points[split_index + 1][1])],
                      color=ColorRGBA(0, 1, 0, 0.7), line_width=0.005)

    return np.argmax(distances) + 1

"""
Calculates the maximal possible speed which can be targeted before the car hits the breaking point.
distance: remaining distance to the curve entry
current_speed: current speed of the car
target_speed: target speed which should be reached at curve entry
acceleration: typical acceleration of the car
decceleration: typical decceleration of the car
"""
def calc_max_speed(distance, current_speed, target_speed, acceleration, decceleration):
    return math.sqrt((2 * distance * acceleration * decceleration + current_speed**2 * decceleration + target_speed**2 * acceleration) / (acceleration + decceleration))

"""
Calculates the distance until the car has to break before a curve entry.
distance: remaining distance to the curve entry
current_speed: current speed of the car
target_speed: target speed which should be reached at curve entry
acceleration: typical acceleration of the car
decceleration: typical decceleration of the car
"""
def calc_braking_distance(distance, current_speed, target_speed, acceleration, decceleration):
    return (2 * distance * acceleration + current_speed**2 - target_speed**2) / (2 * acceleration + 2 * decceleration)

"""
Calculates the maximal speed which can be targeted in a curve.
friction: friction of the car on the ground
radius: radius of the curve
"""
def calc_max_curve_speed(friction, radius):
    return math.sqrt(friction * 9.81 * radius)


# def calc_steering_radius(steering_angle):
#     if steering_angle == 0:
#         return 10000
#     return CAR_LENGTH / math.sin(steering_angle * MAX_STEERING_ANGLE)


# def calc_max_steering_angle():
#     if current_speed == 0:
#         return 1
#     intermediate_result = (FRICTION * 9.81 * CAR_LENGTH) / (current_speed**2)
#     if intermediate_result >= 1:
#         return 1
#     angle_rad = math.asin(intermediate_result)
#     return map(0, MAX_STEERING_ANGLE, 0, 1, angle_rad)


# def calc_steering_angle(steering_radius):
#     angle_rad = math.asin(CAR_LENGTH / steering_radius)
#     return map(0, MAX_STEERING_ANGLE, 0, 1, angle_rad)


# def calc_angle_moving_average():
#     if len(last_angle_moving_average[0]) == 0:
#         return 0
#     return sum(last_angle_moving_average[0]) / len(last_angle_moving_average[0])


# def add_angle_moving_average(angle):
#     global last_angle_moving_average
#     if len(last_angle_moving_average[0]) >= last_angle_moving_average[1]:
#         last_angle_moving_average[0] = last_angle_moving_average[0][1:]
#     last_angle_moving_average[0].append(angle)

"""
Returns a predicted point and distance to this point which is a possible future position if the car would drive straight.
This position is important to calculate the error for the pid-controller.
"""
def calc_predicted_car_position():
    prediction_distance = min(0.1 + last_speed * 0.35, 2)
    # steering_radius = calc_steering_radius(calc_angle_moving_average())
    # circle_section_angle = prediction_distance / steering_radius
    # print steering_radius, circle_section_angle, Point(-steering_radius * math.cos(circle_section_angle) + steering_radius, abs(steering_radius * math.sin(circle_section_angle))), prediction_distance
    # return Point(-steering_radius * math.cos(circle_section_angle) + steering_radius, abs(steering_radius * math.sin(circle_section_angle))), prediction_distance
    return Point(0, prediction_distance), prediction_distance


# def calc_circle_intersections(circle_a, circle_b):
#     # calc vector between the two circles
#     ab = Point(circle_b.center.x - circle_a.center.x, circle_b.center.y - circle_a.center.y)
#     distance = math.sqrt(ab.x**2 + ab.y**2)
#     if distance == 0:
#         # no distance between centers
#         return []
#     x = (circle_a.radius**2 + distance**2 - circle_b.radius**2) / (2 * distance)
#     y = circle_a.radius**2 - x**2
#     if y < 0:
#         # no intersection
#         return []
#     elif y > 0:
#         y = math.sqrt(y)
#     # compute unit vectors
#     ex = Point(ab.x / distance, ab.y / distance)
#     ey = Point(-ex.y, ex.x)
#     # compute intersection
#     Q1 = Point(circle_a.center.x + x * ex.x, circle_a.center.y + x * ex.y)
#     if y == 0:
#         # only one intersection
#         return [Q1]
#     Q2 = Point(Q1.x - y * ey.x, Q1.y - y * ey.y)
#     Q1 = Point(Q1.x + y * ey.x, Q1.y + y * ey.y)
#     return [Q1, Q2]


# def calc_circle_tangent(circle_a, outside_point):
#     distance = math.sqrt((circle_a.center.x - outside_point.x)**2 + (circle_a.center.y - outside_point.y)**2) / 2
#     circle_b = Circle(Point((circle_a.center.x - outside_point.x) / 2, (circle_a.center.y - outside_point.y) / 2), distance)
#     intersections = calc_circle_intersections(circle_a, circle_b)
#     intersections.sort(key=lambda intersection: intersection.y, reverse=True)
#     if len(intersections) > 0:
#         return intersections[0]
#     return None


"""
Calculates the determinant of point_c and a line between point_a and point_b.
If the return value is greater than 0 point_c is on the right side, when it is 
zero it is on the line and if it is smaller than zero it is on the left side. 
"""
def which_side_of_line(point_a, point_b, point_c):
    return (point_c[0] - point_a[0]) * (point_b[1] - point_a[1]) - (point_c[1] - point_a[1]) * (point_b[0] - point_a[0])


"""
Returns true if all points are on one side of the line between point_a and point_b.
The parameter demanded_side indicates on which side all points should be.
demanded_side: 1 -> right side, -1 -> left side
"""
def all_points_on_one_side(points, point_a, point_b, demanded_side):
    for point in points:
        if point is not point_b:
            if which_side_of_line(point_a, point_b, point) < 0 and demanded_side > 0:
                return False
            elif which_side_of_line(point_a, point_b, point) > 0 and demanded_side < 0:
                return False
    return True

"""
Returns the point for which all other points are on the demanded_side from a line between the car [0, 0] and this point.
demanded_side: 1 -> right side, -1 -> left side
"""
def calc_polygon_tangent_point(points, demanded_side):
    for point in points:
        if all_points_on_one_side(points, [0, 0], point, demanded_side):
            return point
    return None


# def calc_shortest_distance_to_vector(p1, p2, p3):
#     return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)


# def calc_shortest_distance_to_points(point_a, point_b, points):
#     return min([calc_shortest_distance_to_vector(np.array(point_a), np.array(point_b), np.array(point)) for point in points])

"""
Calculates the point which is the end of a vector which is orthogonal to a line between point_a and point_b.
This vector is safety_factor * CAR_WIDTH long and is on the demanded side.
point_a: start point of the line
point_b: end point of the line
demanded_side: 1 -> right side, -1 -> left side
safety_factor: result point should be safety_factor times the car width away from the line between point_a and point_b.
"""
def calc_orthogonal_point(point_a, point_b, demanded_side, safety_factor):
    distance = math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)
    if demanded_side > 0:
        return Point(point_b[0] - point_b[1] / distance * CAR_WIDTH * safety_factor, point_b[1] + point_b[0] / distance * CAR_WIDTH * safety_factor)
    else:
        return Point(point_b[0] + point_b[1] / distance * CAR_WIDTH * safety_factor, point_b[1] - point_b[0] / distance * CAR_WIDTH * safety_factor)

"""
Calculates the target car position which is used to calculate the error for the pid-controller.
predicted_car_position: Calculated with function calc_predicted_car_position
curve_type: type of the curve which is behind the curve entry (left, right)
left_circle: circle of the left wall
right_circle: circle of the right wall
upper_circle: circle of the upper wall
left_wall: points of the left wall
right_wall: points of the right wall
remaining_distance: remaining distance to the curve entry point
"""
def calc_target_car_position(predicted_car_position, curve_type, left_circle, right_circle, upper_circle, left_wall, right_wall, remaining_distance):
    left_point = left_circle.get_closest_point(predicted_car_position)
    right_point = right_circle.get_closest_point(predicted_car_position)
    central_point = Point((left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2)
    track_width = abs(left_point.x - right_point.x)

    target_position = central_point

    if curve_type == CURVE_TYPE_LEFT:
        target_position = Point(central_point.x + 0.0, central_point.y)
        if predicted_car_position.y > remaining_distance and remaining_distance is not None and upper_circle is not None:
            upper_point = upper_circle.get_closest_point(predicted_car_position)
            target_position = Point(
                (upper_point.x - track_width / 2),
                (left_point.y + right_point.y) / 2)
    elif curve_type == CURVE_TYPE_RIGHT:
        target_position = Point(central_point.x - 0.0, central_point.y)
        if predicted_car_position.y > remaining_distance and remaining_distance is not None and upper_circle is not None:
            upper_point = upper_circle.get_closest_point(predicted_car_position)
            target_position = Point(
                (upper_point.x + track_width / 2),
                (left_point.y + right_point.y) / 2)

    show_line_in_rviz(2, [left_point, right_point],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)

    left_tangent_point = None
    right_tangent_point = None
    # left curve
    if left_circle.center.x < 0 and right_circle.center.x < 0 and (left_circle.radius < 1000 or right_circle.radius < 1000): #calc_smallest_distance_to_points([0, 0], [target_position.x, target_position.y], left_wall) < CAR_WIDTH
        left_tangent_point = calc_polygon_tangent_point(left_wall, -1)
        if left_tangent_point is not None:
            target_position = calc_orthogonal_point([0, 0], left_tangent_point, -1, 2.0)
    # right curve
    if left_circle.center.x > 0 and right_circle.center.x > 0 and (left_circle.radius < 1000 or right_circle.radius < 1000): #calc_smallest_distance_to_points([0, 0], [target_position.x, target_position.y], right_wall) < CAR_WIDTH
        right_tangent_point = calc_polygon_tangent_point(right_wall, 1)
        if right_tangent_point is not None:
            target_position = calc_orthogonal_point([0, 0], right_tangent_point, 1, 2.0)


    if left_tangent_point is not None:
        show_line_in_rviz(20, [Point(0, 0), Point(left_tangent_point[0], left_tangent_point[1])],
                        color=ColorRGBA(1, 1, 1, 1.0))
    else:
        delete_marker(20)
    if right_tangent_point is not None:
        show_line_in_rviz(21, [Point(0, 0), Point(right_tangent_point[0], right_tangent_point[1])],
                        color=ColorRGBA(0.5, 0.5, 0.5, 1.0))
    else:
        delete_marker(21)

    return target_position


# def calc_closest_point_to_vector(points, p1, p2):
#     distances = np.array([calc_shortest_distance_to_vector(p1, p2, point) for point in points])
#     closest_point_index_to_vector = np.argmin(distances)
#     return points[closest_point_index_to_vector]


# def angle_between(v1, v2):
#     """ Returns the angle in radians between vectors 'v1' and 'v2':: """
#     angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
#     if v1[0] < v2[0]:
#         return -angle
#     return angle


# def rotate_origin_only(points, target_line):
#     """Only rotate a point around the origin (0, 0)."""
#     radians = angle_between(np.array([0, 1]), target_line)
#     print radians

#     x_points = points[:, 0] * np.cos(radians) + points[:, 1] * np.sin(radians)
#     y_points = -points[:, 0] * np.sin(radians) + points[:, 1] * np.cos(radians)

#     return np.stack((x_points, y_points), axis=1)

"""
Determines speed and steering angle for the car based on a wall following algorithm.
left_circle: circle of the left wall
right_circle: circle of the right wall
upper_circle: circle of the upper wall
left_wall: points of the left wall
right_wall: points of the right wall
curve_type: type of the curve which is behind the curve entry (left, right)
remaining_distance: remaining distance to the curve entry point
delta_time: passed time since the last call of follow_walls
"""
def follow_walls(left_circle, right_circle, upper_circle, left_wall, right_wall, curve_type, remaining_distance, delta_time):
    global last_speed
    predicted_car_position, prediction_distance = calc_predicted_car_position()
    target_position = calc_target_car_position(predicted_car_position, curve_type, left_circle, right_circle, upper_circle, left_wall, right_wall, remaining_distance)

    distance_to_target = math.sqrt(target_position.x**2 + target_position.y**2)

    error = (target_position.x - predicted_car_position.x) / distance_to_target
    if math.isnan(error) or math.isinf(error):
        error = 0

    steering_angle = pid.update_and_get_correction(
        error, delta_time)

    radius = min(left_circle.radius, right_circle.radius)

    speed = calc_max_curve_speed(FRICTION, radius)
    if remaining_distance is not None and upper_circle is not None:
        safety_margin = 0.25
        if remaining_distance < 5:
            safety_margin = 0.05 * remaining_distance
        target_speed = calc_max_curve_speed(FRICTION, upper_circle.radius)
        last_speed = target_speed
        braking_distance = calc_braking_distance(remaining_distance, current_speed, target_speed, CAR_ACCELERATION, CAR_DECCELERATION) + safety_margin
        if remaining_distance > braking_distance:
            speed = min(calc_max_speed(remaining_distance, current_speed, target_speed, CAR_ACCELERATION, CAR_DECCELERATION), speed)
        else:
            speed = target_speed

    steering_angle = steering_angle * map(parameters.high_speed_steering_limit_dead_zone, 1, 1, parameters.high_speed_steering_limit, speed / 25)  # nopep8
    # max_steering_angle = calc_max_steering_angle() # + (speed / 25) * 2
    # if steering_angle < 0:
    #     steering_angle = max(-max_steering_angle, steering_angle)
    # else:
    #     steering_angle = min(steering_angle, max_steering_angle)
    # add_angle_moving_average(steering_angle)

    # if a curve is unexpected steep, the speed may be reduced
    emergency_slowdown = min(1, distance_to_target**2 / prediction_distance**2)
    if emergency_slowdown > 0.7:
        emergency_slowdown = 1
    speed *= emergency_slowdown
    speed = max(2, speed)
    drive(steering_angle, speed)

    show_line_in_rviz(3, [Point(0, 0), predicted_car_position],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    show_line_in_rviz(4, [predicted_car_position,
                          target_position], color=ColorRGBA(1, 0.4, 0, 1))


def handle_scan(laser_scan, delta_time):
    if parameters is None:
        return

    points = get_scan_as_cartesian(laser_scan)

    if points.shape[0] == 0:
        rospy.logwarn("Skipping current laser scan message since it contains no finite values.")  # nopep8
        return

    split = find_left_right_border(points)

    right_wall = points[:split:4, :]
    left_wall = points[split::4, :]

    # min_x_left_wall = min([point[0] for point in left_wall])
    # right_wall = np.array([point for point in right_wall if point[0] > min_x_left_wall])

    # max_x_right_wall = max([point[0] for point in right_wall])
    # left_wall = np.array([point for point in left_wall if point[0] < max_x_right_wall])

    left_circle = Circle.fit(left_wall)
    right_circle = Circle.fit(right_wall)

    # looks what the closest point in front of the car is
    ranges_in_front = get_scans_in_angular_range(laser_scan, -0.01, 0.01)
    if len(ranges_in_front) != 0:
        min_range_in_front = min(ranges_in_front)
    else:
        min_range_in_front = 30

    curve_entry_point = None
    curve_type = None
    if min_range_in_front < 30:
        # With radius_proportions can be checked whether the car is approaching a curve or is on a straight part of the track.
        radius_proportions_left = left_circle.radius / right_circle.radius
        radius_proportions_right = right_circle.radius / left_circle.radius
        if radius_proportions_left > 1.2 and right_circle.center.x < 0:
            curve_entry_point = left_wall[np.argmax(left_wall[:, 1])]
            upper_wall = np.array([point for point in right_wall if point[1] >= curve_entry_point[1] - 1.5])
            right_wall = np.array([point for point in right_wall if point[1] < curve_entry_point[1]])
            right_circle = Circle.fit(right_wall)
            curve_type = CURVE_TYPE_LEFT
        elif radius_proportions_right > 1.2 and right_circle.center.x > 0:
            curve_entry_point = right_wall[np.argmax(right_wall[:, 1])]
            upper_wall = np.array([point for point in left_wall if point[1] >= curve_entry_point[1] - 1.5])
            left_wall = np.array([point for point in left_wall if point[1] < curve_entry_point[1]])
            left_circle = Circle.fit(left_wall)
            curve_type = CURVE_TYPE_RIGHT

    upper_circle = None
    remaining_distance = None
    if curve_entry_point is not None and len(upper_wall) > 0:
        remaining_distance = math.sqrt(curve_entry_point[0]**2 + curve_entry_point[1]**2)
        upper_circle = Circle.fit(upper_wall)
        show_line_in_rviz(
            6, [Point(-2, remaining_distance), Point(2, remaining_distance)], color=ColorRGBA(0.2, 0.5, 0.8, 1))
        show_circle_in_rviz(upper_circle, upper_wall, 7, ColorRGBA(0, 1, 1, 1))
    else:
        delete_marker(6)
        delete_marker(7)

    follow_walls(left_circle, right_circle, upper_circle, left_wall, right_wall, curve_type, remaining_distance, delta_time)

    show_circle_in_rviz(left_circle, left_wall, 0, ColorRGBA(0.5, 1, 1, 1))
    show_circle_in_rviz(right_circle, right_wall, 1, ColorRGBA(0, 1, 1, 1))


last_scan = None


def laser_callback(scan_message):
    global last_scan # total_scan_count, total_scan_time
    t_start = time.time()
    scan_time = scan_message.header.stamp.to_sec()
    if last_scan is not None and abs(scan_time - last_scan) > 0.0001 and scan_time > last_scan:  # nopep8
        delta_time = scan_time - last_scan
        handle_scan(scan_message, delta_time)

    last_scan = scan_time
    # t_delta = time.time() - t_start
    # total_scan_time += t_delta
    # total_scan_count += 1
    # if total_scan_count % 40 == 0:
    #     print "mean_scan_time:", total_scan_time / float(total_scan_count) * 1000.0, "ms", "scan_time:", delta_time, "s"


def speed_callback(speed_message):
    global current_speed
    current_speed = speed_message.wheel_speed
    # print current_speed - speed_message.car_speed

"""
Callback function for the /commands/controlled_drive_param to get an approximated speed of the car from the acceleration_controller.
"""
def controlled_drive_param_callback(drive_param):
    global current_speed
    current_speed = drive_param.velocity


def dynamic_configuration_callback(config, level):
    global parameters
    parameters = Parameters(config)
    pid.p = parameters.controller_p
    pid.i = parameters.controller_i
    pid.d = parameters.controller_d
    return config


rospy.init_node('wallfollowing3', anonymous=True)
parameters = None
pid = PIDController(1, 1, 1)

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback, queue_size=1)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback, queue_size=1)
rospy.Subscriber(TOPIC_CONTROLLED_DRIVE_PARAM, drive_param, controlled_drive_param_callback, queue_size=1)
drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

Server(wallfollowing3Config, dynamic_configuration_callback)

while not rospy.is_shutdown():
    rospy.spin()