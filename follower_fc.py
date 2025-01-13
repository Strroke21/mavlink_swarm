#!/usr/bin/env python3

import math
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2
from time import sleep,time
import pyudev
import subprocess

#------- FCU connection ---------
fcu_addr =  '/dev/ttyACM0' #'tcp:127.0.0.1:5773'
form_alt = 10 #formation altitude
master_addr =  'udp:0.0.0.0:14555' #'tcp:127.0.0.1:5783'
stream_rate = 100 #in Hz

#----------- formation1 Variables-----------
f1_distance = 10 #10 m in right
f1_angle = 90 #angular pos relative to leader
f1_min_pwm = 950
f1_max_pwm = 1020

#----------- formation2 variables ---------------

f2_distance = 15
f2_angle = 45
f2_min_pwm = 1150
f2_max_pwm = 1250

#------------ formation3 variables ------------
f3_distance = 15
f3_angle = 135
f3_min_pwm = 1350
f3_max_pwm = 1450


#unchanged
counter = 0
tar_alt = 20
land_alt = 5

kp = 0.5 #proportional gain

#-------------- Functions -----------------------

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE","","LAND"]
    if mode in modes:
        mode_id = modes.index(mode)
    else:
        mode_id = 12
    vehicle.mav.set_mode_send(vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
    
def drone_takeoff(vehicle, altitude):
    # Send MAVLink command to takeoff
    vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altitude)

def get_global_position(vehicle):
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat/1e7 # lat
    lon = msg.lon/1e7 # lon
    alt = msg.alt/1000  # alt
    return [lat,lon,alt]

def goto_waypoint(vehicle,latitude, longitude, altitude):
    msg = vehicle.mav.set_position_target_global_int_encode(
        time_boot_ms=10,
        target_system=vehicle.target_system,       # Target system (usually 1 for drones)
        target_component=vehicle.target_component,    # Target component (usually 1 for drones)
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference for the coordinate system
        type_mask=0b0000111111111000,        # Bitmask to indicate which dimensions should be ignored (0b0000111111111000 means all ignored except position)
        lat_int=int(latitude * 1e7),       # Latitude in degrees * 1e7 (to convert to integer)
        lon_int=int(longitude * 1e7),      # Longitude in degrees * 1e7 (to convert to integer)
        alt=altitude,           # Altitude in meters (converted to millimeters)
        vx=0,                         # X velocity in m/s (not used)
        vy=0,                         # Y velocity in m/s (not used)
        vz=0,                         # Z velocity in m/s (not used)
        afx=0, afy=0, afz=0,                   # Accel x, y, z (not used)
        yaw=0, yaw_rate=0                       # Yaw and yaw rate (not used)
    )
    vehicle.mav.send(msg)


def relative_pos(lat, lon, distance, heading, follower_heading):
        # Convert heading to radians and calculate new heading
    EARTH_RADIUS = 6371000
    heading_rad = math.radians(heading)
    new_heading_rad = heading_rad + math.radians(follower_heading)
    
    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    # Calculate the new latitude and longitude
    delta_lat = distance * math.cos(new_heading_rad) / EARTH_RADIUS
    delta_lon = distance * math.sin(new_heading_rad) / (EARTH_RADIUS * math.cos(lat_rad))
    
    new_lat_rad = lat_rad + delta_lat
    new_lon_rad = lon_rad + delta_lon
    
    # Convert the new latitude and longitude back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)
    
    return new_lat, new_lon


def formation(angle,distance,form_alt):
    counter = 0
    while True:
        ######## leader ######
        #data =  mavlink_data_queue.get()
        current_pos = master_data(master,"GLOBAL_POSITION_INT")
        if current_pos:
            counter+=1
            lat_lead = current_pos['lat']/1e7
            lon_lead = current_pos['lon']/1e7
            lead_yaw = (current_pos['hdg'])/100
            print(f"Leader Heading: {lead_yaw} deg.")
            ######### follower1 ######
            yaw1 = angle #on the right
            dist1 = distance#change this distance to add new follower
            lat1, lon1 = relative_pos(lat_lead, lon_lead, dist1, lead_yaw, yaw1)
            print(lat1, lon1)
            ##### send msg to follower1 drone #############

            if counter==1:
                goto_waypoint(follower1,lat1,lon1,form_alt) #altitude 1
                print("formation command sent.")
                sleep(0.15)
    
            dist_to_target = distance_between(follower1,lat1,lon1)
            print(f"distance to formation: {dist_to_target:.2f} m.")
            if dist_to_target<=2:
                sleep(1)
                break

        else:
            print("no data retrieved from the leader.")



def distance_to_leader(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

def get_local_position(vehicle):
    msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    pos_x = msg.x # Degrees
    pos_y = msg.y  # Degrees
    pos_z = msg.z  # Meters
    return [pos_x,pos_y,pos_z]

def send_velocity_setpoint(vehicle, vx, vy, vz):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        0, 0, 0,                    # position (not used)
        vx, vy, vz,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )

def angle_difference_with_heading(lat1, lon1, heading, lat2, lon2): # angle difference relative to origin lat, lon, heading and current lat, lon

    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Compute the difference in the longitudes
    delta_lon = lon2 - lon1

    # Calculate the bearing from the reference point to the second location
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    bearing_to_point = math.atan2(x, y)

    # Convert bearing from radians to degrees
    bearing_to_point = math.degrees(bearing_to_point)

    # Normalize the bearing to a value between 0 and 360 degrees
    bearing_to_point = (bearing_to_point + 360) % 360

    # Calculate the angle difference relative to the reference heading
    angle_diff = (bearing_to_point - heading + 360) % 360

    return angle_diff

def distance_between(vehicle,leader_lat,leader_lon):
    
    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - leader_lat)
    dlon = radians(current_lon - leader_lon)
    a = sin(dlat / 2)**2 + cos(radians(leader_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters

def send_position_setpoint(vehicle, pos_x, pos_y, pos_z):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b110111111000,        # type_mask (only for postion)
        pos_x, pos_y, pos_z,   # position 
        0, 0, 0,                 # velocity in m/s (not used)
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)0
    )


def geo_distance_components(vehicle, lat2, lon2):

    lat1 = get_global_position(vehicle)[0]
    lon1 = get_global_position(vehicle)[1]
    # Earth's radius in meters
    R = 6371000

    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Calculate differences
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    # North-South component (y): R * delta_lat
    y = R * delta_lat

    # East-West component (x): R * delta_lon * cos(mean_latitude)
    mean_lat = (lat1 + lat2) / 2
    x = R * delta_lon * math.cos(mean_lat)

    return x, y


def control_xy(vehicle, vx, vy, x, y, kp):

    dx = kp*x
    dy = kp*y
    cx = vx + dx
    cy = vy + dy
    send_velocity_setpoint(vehicle,cx,cy,0)
    print(f"correction velocity X: {cx} Correction Velocity Y: {cy}")
    

def change_formation(angle,distance,form_alt,tar_alt,min_pwm,max_pwm):

    lat,lon,_ = get_global_position(follower1)
    goto_waypoint(follower1,lat,lon,form_alt) #formation altitude
    sleep(5)
    formation(angle,distance,form_alt)
    nlat,nlon,_ = get_global_position(follower1)
    goto_waypoint(follower1,nlat,nlon,tar_alt) #final altitude
    #------ checkpoint for final altitude ------------------
    #target_alt_check(tar_alt)
    sleep(5)

    while True:
        pos = master_data(master,"GLOBAL_POSITION_INT")
        rc_chan = master_data(master,"RC_CHANNELS")
        print(pos,rc_chan)
        if pos:
            
            #fetch leader velocity in all direction in m/s
            lead_vx = pos['vx']/100
            lead_vy = pos['vy']/100
            lead_vz = pos['vz']/100
            lead_lat = pos['lat']/1e7
            lead_lon = pos['lon']/1e7
            lead_yaw = pos['hdg']/100
            altitude = (master_data(master,"DISTANCE_SENSOR")['current_distance'])/100 #rangefinder altitude
            send_velocity_setpoint(follower1,lead_vx,lead_vy,0) #sending only vx and vy
            print(f"leader vx: {lead_vx} m/s leader vy: {lead_vy} m/s leader vz: {lead_vz} m/s")

            form_lat, form_lon = relative_pos(lead_lat,lead_lon,distance,lead_yaw, angle)
            x,y = geo_distance_components(follower1, form_lat, form_lon)

            if (abs(x) > 0.5) and (abs(y)>0.5):

                control_xy(follower1,lead_vx,lead_vy,x,y,kp=0.5)  #position correction

            print(f"x_dist_to_goal: {x:.2f}m. y_dist_to_goal {y:.2f}m.")

            if abs(altitude) <= land_alt: #this altitude should be local not relative just for now using this
                VehicleMode(follower1,"LAND")
                sleep(1)
                break

            elif (int(rc_chan['chan6_raw'])>1900):
                VehicleMode(follower1,"RTL")
                print("Vehicle in RTL mode...")
                print("Returning to home...")
                sleep(1)
                exit()

            elif (min_pwm > int(rc_chan['chan11_raw'])) or (int(rc_chan['chan11_raw'])>max_pwm):
                print("Changing Formation...")
                break

            else:
                pass

        else:
            print("no data received!!!")


def arming_check():
    #-------------------------------------arming checkpoint for follower from leader------------------------------
    while True:

        data = master_data(master,"DISTANCE_SENSOR")
        print(data)
        if data:
            alt = data['current_distance']/100 #rangefinder altitude (m.)
            if alt >=10:
                break

            print(f"Leader Altitude: {alt:.2f} m.")
        else:
            print("no data received from leader")


def target_alt_check(alt):
    #--------------------- checkpoint for target altitude --------------
    while True:
        msg = follower1.recv_match(type='DISTANCE_SENSOR', blocking=True) #rangefinder altitude
        altitude = abs(msg.current_distance/100)
        print(f"Altitude: {altitude} m.")
        if altitude>(alt-1):
            sleep(1)
            print("Target altitude reached.")
            break


def master_data(master,msg_type):

    msg = master.recv_match(type=msg_type,blocking=True)

    if msg:
        data = msg.to_dict()
        return data
    

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

master = connect(master_addr) #stream rate set by comm
print("Master Connected...")
#enable_data_stream(master,stream_rate) #stream already enabled by master script

arming_check()
#---------- follower connection ---------
follower1 = connect(fcu_addr)#('tcp:127.0.0.1:5763')
print("follower1 connected.")
enable_data_stream(follower1,stream_rate)
#----------guided mode ----------
VehicleMode(follower1,"GUIDED")
print("follower1 in GUIDED mode")
sleep(1)

#------- arm --------
arm(follower1)
print("arming the follower1")
sleep(1)
#---------- guided takeoff -----------
drone_takeoff(follower1,form_alt)
print("taking off follower1")
#target_alt_check(form_alt)
sleep(10)
#-------------- arming and takeoff checkpoint 

#------------------- main ------------------
while True:

    pos = master_data(master,"GLOBAL_POSITION_INT")
    rc_chan = master_data(master,"RC_CHANNELS")
    print(rc_chan, pos)

    if (f1_min_pwm < int(rc_chan['chan11_raw']) < f1_max_pwm):
        change_formation(f1_angle,f1_distance,form_alt,tar_alt,f1_min_pwm,f1_max_pwm)


    elif (f2_min_pwm < int(rc_chan['chan11_raw']) < f2_max_pwm):
        change_formation(f2_angle,f2_distance,form_alt,tar_alt,f2_min_pwm,f2_max_pwm)


    elif (f3_min_pwm < int(rc_chan['chan11_raw']) < f3_max_pwm):
        change_formation(f3_angle,f3_distance,form_alt,tar_alt,f3_min_pwm,f3_max_pwm)

    else:
        print("Waiting for data...")






