import sys
from struct import *
from hornet_acoustics_library import *

def convert_hexstring_to_float(packet_hex):
    # Decode last 4 items as hex to float
    packet_hex = items[-4:]
    packet_bytes = bytes(map(lambda g: int(g, 16), packet_hex))
    packet_float = unpack('<f', packet_bytes)
    return packet_float
    
def convert_hexstring_to_integer(packet_hex):
    # Decode items as hex to integer
    packet_bytes = bytes(map(lambda g: int(g, 16), packet_hex))
    packet_short = unpack('<H', packet_bytes)
    return packet_short[0]

previous_depth_sensor = 2

acoustics = Acoustics(hx=43.5, hy=47, hz=29)

while True:
    line = sys.stdin.readline().strip()
    # can0  302   [8]  00 00 00 00 00 00 00 00
    if line.startswith("can0  302"): # Hydrophone 45 kHz
        items = line.split(' ')
        hex_items = items[-8:]
        print("\n>>", line, hex_items)
        
        # Receive raw values from CAN
        h1 = convert_hexstring_to_integer(hex_items[0:2])
        h2 = convert_hexstring_to_integer(hex_items[2:4])
        h3 = convert_hexstring_to_integer(hex_items[4:6])
        h4 = convert_hexstring_to_integer(hex_items[6:8])

        hydrophone_raw_timing = [h1, h4, h2, h3] # [ref, x, y, z]
        print("Raw timing (ticks):", hydrophone_raw_timing)

        hydrophone_timing = [h1-h1, h4-h1, h2-h1, h3-h1] # [ref, x, y, z]
        print("TDOA timing (ticks):", hydrophone_timing)

        # Update values into library
        acoustics.update(hydrophone_timing)

        # Get calculated values
        hydrophone_tdoa_distances = acoustics.tdoa_distance_list
        print("TDOA Distance (cm):", hydrophone_tdoa_distances)

        # if hydrophone_tdoa_distances[1] > hx or hydrophone_tdoa_distances[2] > hy or hydrophone_tdoa_distances[3] > hz:
        #     print("Possible error")
        # else:

        # Calculate angle of arrival
        angle_elevation = acoustics.get_elevation_angle()
        print("Elevation angle (deg):", angle_elevation)
        angle_bearing = acoustics.get_bearing_angle(account_for_elevation=True)
        print('Bearing angle (deg):', angle_bearing)

        # Estimate distance to pinger using depth sensor
        depth_from_floor = 2 - previous_depth_sensor # Sensor distance is from the surface (in meters)
        target_extimated_radius = acoustics.estimate_distance_to_pinger(depth=depth_from_floor)
        print('Estimated distance to pinger (meters)', target_extimated_radius)

        # End
        print("="*80)
    
    elif line.startswith("can0  105"): # Depth sensor
        items = line.split(' ')
        packet_hex = items[-4:]
        packet_float = convert_hexstring_to_float(packet_hex)
        print("\rDepth:", packet_float, end="")
        previous_depth_sensor = packet_float[0]
    
    else:
        #print("xx", line)
        pass







