import numpy as np

class Acoustics:

    def hydrophone_positions_array(self, hx, hy, hz):
        positions = np.array([
            [0, 0, 0],
            [hx, 0, 0],
            [0, hy, 0],
            [0, 0, hz],
        ])
        return positions

    """
    Convert the TDOA raw timing from the filter board
    into centimeters. Note that the sample frequency is hardcoded
    """
    def convert_timing_to_tdoa_distances(self, raw_timing_list):
        SPEED_OF_SOUND = 1500 # meter/sec
        GOERTZEL_SAMPLE_FREQ = 500000 # hertz = 1/sec
        raw_timing_list = np.array(raw_timing_list)    
        seconds_list = raw_timing_list * (1/GOERTZEL_SAMPLE_FREQ)
        centimeters_list = seconds_list * SPEED_OF_SOUND * 100;
        return centimeters_list

    def __init__(self, hx, hy, hz):
        self.hx = hx
        self.hy = hy
        self.hz = hz
        self.hydrophone_positions = self.hydrophone_positions_array(hx, hy, hz)

    def update(self, raw_timing_list):
        (ref, x, y, z) = raw_timing_list
        zeroed_timing_list = [ref-ref, x-ref, y-ref, z-ref]
        self.tdoa_distance_list = self.convert_timing_to_tdoa_distances(zeroed_timing_list)

    """
    If we have depth sensor, we can estimate radius of pinger
    Distance estimate is more accurate as the pinger get closer to the vehicle
    --> depth[centimeters] / tan(angle_elevation)
    returns centimeters
    """
    def estimate_distance_to_pinger(self, depth):
        degrees_elevation = self.get_elevation_angle()
        if degrees_elevation < 90:
            target_extimated_radius = abs(depth) / np.tan(np.radians(degrees_elevation))
        else:
            target_extimated_radius = 0
        return target_extimated_radius

    """
    Use stacked hydrophone pair to extimate angle of elevation.
    Angle is more accurate as the pinger get closer to the vehicle
    --> angle_elevation = arcsin(tdoa_vertical / distance_vertical)
    """
    def get_elevation_angle(self):
        delta_vertical_hydrophones = self.tdoa_distance_list[3] - self.tdoa_distance_list[0]
        hz = np.linalg.norm(self.hydrophone_positions[3] - self.hydrophone_positions[0])
        angle_elevation = np.arccos(delta_vertical_hydrophones / hz)
        angle_elevation = 90 - np.degrees(angle_elevation)
        return angle_elevation

    """
    For bearing angle, we take the sequence of hydrophone trigger timings to determing the quadrant
    
    The calculated bearing angle uses the TDOA between the hydrophone pair, 
    with normalisation done using the elevation angle to get the projected length
    (TDOA provides adjacent, while hypotheneus should be used which is parallel to the poolbed).
    """
    def get_bearing_angle(self, account_for_elevation=True):
        # Difference in cemtimeters.
        delta_tdoa_distances = self.tdoa_distance_list - min(self.tdoa_distance_list)

        # Get hydrophone index in terms of which one is triggered first
        trigger_sorted_index = sorted(list(range(len(self.tdoa_distance_list[:-1]))), key=lambda i: (self.tdoa_distance_list[:-1])[i])
        trigger_sorted_index = tuple(trigger_sorted_index)

        # Find quadrant based on trigger sequence
        quadrant = 'unknown'
        # 1A : RXY
        # 1B : RYX
        # 2A : YRX, where R < 0.5(X+Y) @
        # 2B : YRX, where R > 0.5(X+Y) @
        # 3A : YXR
        # 3B : XYR
        # 4A : XRY, where R > 0.5(X+Y) @
        # 4B : XRY, where R < 0.5(X+Y) @

        # @ Actually midpoint is not exactly middle at 0.5 since hx != hy
        # Let m be the midpoint ratio
        #    sin 45deg = m*d/hy = (1-m)*d/hx
        #    m*(1+hx/hy) = 1
        midpoint_ratio = 1 / (1 + self.hx / self.hy)

        if trigger_sorted_index == (0, 1, 2):
            quadrant = '1A'
        elif trigger_sorted_index == (0, 2, 1):
            quadrant = '1B'
        elif trigger_sorted_index == (2, 0, 1):
            if delta_tdoa_distances[0] < midpoint_ratio*(delta_tdoa_distances[1] + delta_tdoa_distances[2]):
                quadrant = '2A'
            else:
                quadrant = '2B'
        elif trigger_sorted_index == (2, 1, 0):
            quadrant = '3A'
        elif trigger_sorted_index == (1, 2, 0):
            quadrant = '3B'
        elif trigger_sorted_index == (1, 0, 2):
            if delta_tdoa_distances[0] > midpoint_ratio*(delta_tdoa_distances[1] + delta_tdoa_distances[2]):
                quadrant = '4A'
            else:
                quadrant = '4B'

        # Calculate based on most accurate hydrophone pairs
        def find_tdoa_distance(first, last, include_elevation=True):
            distance = delta_tdoa_distances[last] - delta_tdoa_distances[first]
            degrees_elevation = self.get_elevation_angle()

            if include_elevation and degrees_elevation >= 0:
                return distance / np.cos(np.radians(degrees_elevation))
            else:
                return distance
            

        def angle_between(first, last):
            # calculate angle between a pair of trigger hydrophones
            tdoa_length = find_tdoa_distance(first, last, account_for_elevation)
            hydrophone_length = np.linalg.norm(self.hydrophone_positions[last] - self.hydrophone_positions[first])
            ratio = min(1, tdoa_length/hydrophone_length)
            return np.degrees(np.arccos(ratio))

        if quadrant == '1A':
            angle = 0 + angle_between(0, 2)
        elif quadrant == '1B':
            angle = 90 - angle_between(0, 1)
        elif quadrant == '2A':
            angle = 90 + angle_between(0, 1)
        elif quadrant == '2B':
            angle = 180 - angle_between(2, 0)
        elif quadrant == '3A':
            angle = 180 + angle_between(2, 0)
        elif quadrant == '3B':
            angle = 270 - angle_between(1, 0)
        elif quadrant == '4A':
            angle = 270 + angle_between(1, 0)
        elif quadrant == '4B':
            angle = 360 - angle_between(0, 2)
        else:
            angle = None

        return {'angle': angle, 'quadrant': quadrant}
