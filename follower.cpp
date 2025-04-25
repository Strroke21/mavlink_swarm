#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavlink/common/mavlink.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip> 
#include <cmath>
#include <atomic>  // For thread-safe flag
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

using namespace mavsdk;

//varibles
double dist_to_leader = 10.0; // Distance in meters
double angle_to_leader = 90.0; // Heading in degrees (for follower formation)
int counter = 0;
float target_alt = 20.0; // Target altitude in meters
float pos_tolerence = 2.0; // Position tolerence in meters
float kp = 1.5; // Proportional gain for position control
float form_alt = 10.0; // Formation altitude in meters


std::vector<double> get_rfd900x_data(const std::string& serial_port) {
    std::vector<double> leader_data;
    int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        std::cerr << "Failed to open serial port: " << serial_port << std::endl;
        return leader_data;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(fd);
        return leader_data;
    }

    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 2;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &tty);

    // Enter AT command mode
    write(fd, "+++", 3);
    usleep(1000000);

    // Send AT&L
    write(fd, "AT&L\r\n", 6);
    usleep(200000);

    char buffer[256];
    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
        buffer[n] = '\0';
        std::string response(buffer);
        std::cout << "Raw response: " << response << std::endl;

        // Simulated: assume response is CSV like: "37.7749,-122.4194,45.0,1.2,0.8,-0.1\n"
        std::stringstream ss(response);
        std::string token;

        while (std::getline(ss, token, ',')) {
            try {
                leader_data.push_back(std::stod(token));
            } catch (...) {
                std::cerr << "Warning: Failed to convert token: " << token << std::endl;
            }
        }
    }

    close(fd);
    return leader_data;
}


double distance_between(double current_lat, double current_lon, double leader_lat, double leader_lon) {
    // Convert degrees to radians
    const double R = 6378137.0;  // Earth radius in meters
    double dlat = (current_lat - leader_lat) * M_PI / 180.0;
    double dlon = (current_lon - leader_lon) * M_PI / 180.0;

    // Haversine formula
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(leader_lat * M_PI / 180.0) * std::cos(current_lat * M_PI / 180.0) * 
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // Distance in meters
    double distance = R * c;

    return distance;
}

bool goto_waypoint(System& system, float target_lat, float target_lon, float target_alt)
{
    MavlinkPassthrough mavlink_passthrough{system};
    Telemetry telemetry{system};

    mavlink_message_t msg; // NOT const here

    mavlink_msg_set_position_target_global_int_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        static_cast<uint32_t>(0),
        1, // target system
        1, // target component
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        static_cast<int32_t>(target_lat * 1e7),
        static_cast<int32_t>(target_lon * 1e7),
        target_alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    );

    // Safe to send as const now
    auto result = mavlink_passthrough.send_message(msg);

    if (result != MavlinkPassthrough::Result::Success) {
        std::cerr << "Failed to send waypoint: " << static_cast<int>(result) << std::endl;
        return false;
    }
    
    std::cout << "Goto command sent to reach target waypoint." << std::endl;

    bool target_reached = false;

    while (!target_reached) {
        Telemetry::Position position = telemetry.position();

        double current_lat = position.latitude_deg;
        double current_lon = position.longitude_deg;

        double dist = distance_between(current_lat, current_lon, target_lat, target_lon);
        std::cout << "Distance to target: " << dist << " meters." << std::endl;

        if (dist <= 2.0) {
            std::cout << "Target reached!" << std::endl;
            target_reached = true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return true;

}


void set_velocity(System& system, float vx, float vy, float vz) {
    MavlinkPassthrough mavlink_passthrough{system};

    mavlink_message_t msg;

    mavlink_msg_set_position_target_local_ned_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        static_cast<uint32_t>(0),   // time_boot_ms (can be 0 or from telemetry)
        1,                          // target_system (usually 1)
        1,                          // target_component (usually 1)
        MAV_FRAME_LOCAL_NED,       // coordinate frame
        0b0000111111000111,         // type_mask (only vx, vy, vz are active)
        0.0f, 0.0f, 0.0f,           // x, y, z position (ignored)
        vx, vy, vz,                 // velocity in m/s
        0.0f, 0.0f, 0.0f,           // acceleration (ignored)
        0.0f, 0.0f                  // yaw, yaw_rate (ignored)
    );

    mavlink_passthrough.send_message(msg);
}

std::pair<float, float> relative_pos(double lat, double lon, double distance, double heading, double follower_heading) {
    const double EARTH_RADIUS = 6378137.0; // Earth's radius in meters

    double heading_rad = heading * M_PI / 180.0;
    double new_heading_rad = heading_rad + (follower_heading * M_PI / 180.0);

    // Convert degrees to radians
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // Calculate the difference in latitude and longitude
    double delta_lat = distance * std::cos(new_heading_rad) / EARTH_RADIUS;
    double delta_lon = distance * std::sin(new_heading_rad) / (EARTH_RADIUS * std::cos(lat_rad));

    double new_lat_rad = lat_rad + delta_lat;
    double new_lon_rad = lon_rad + delta_lon;

    // Convert back to degrees
    float new_lat = static_cast<float>(new_lat_rad * 180.0 / M_PI);
    float new_lon = static_cast<float>(new_lon_rad * 180.0 / M_PI);

    return std::make_pair(new_lat, new_lon);
}


bool formation(System& system, double angle_to_leader, double dist_to_leader, double form_alt, double leader_lat, double leader_lon, double leader_yaw){

    int form_counter = 0;
    while (true){
        form_counter++;
        auto [lat, lon] = relative_pos(leader_lat, leader_lon, dist_to_leader, leader_yaw, angle_to_leader);
        std::cout << "Follower position: " << lat << ", " << lon << std::endl;
        
        if (form_counter == 1) {
            std::cout << "Formation command sent to reach target waypoint." << std::endl;
            goto_waypoint(system, lat, lon, form_alt);
            return true;
        }
    }

}

std::pair<double, double> get_global_position(System& system, Telemetry& telemetry) {
    // Get current position (non-blocking snapshot)
    Telemetry::Position position = telemetry.position();
    double current_lat = position.latitude_deg;
    double current_lon  = position.longitude_deg;

    return std::make_pair(current_lat, current_lon);
}

float current_heading(System& system, Telemetry& telemetry){
    // Get current heading (non-blocking snapshot)
    Telemetry::EulerAngle euler_angle = telemetry.attitude_euler();
    float current_heading = euler_angle.yaw_deg;

    if (current_heading < 0) {
        current_heading += 360.0f; // Normalize to [0, 360)
    }

}

bool takeoff(System& system, float takeoff_altitude_m) {
    Action action(system);
    Telemetry telemetry(system);

    // Set desired takeoff altitude
    auto altitude_result = action.set_takeoff_altitude(takeoff_altitude_m);
    if (altitude_result != Action::Result::Success) {
        std::cerr << "Failed to set takeoff altitude: " << altitude_result << std::endl;
        return false;
    }

    // Wait for system to be ready
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Arm the drone
    auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
    }

    std::cout << "Armed successfully." << std::endl;

    // Takeoff
    auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return false;
    }

    std::cout << "Takeoff initiated to " << takeoff_altitude_m << " meters..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return true;
    
}

void geo_distance_components(System& system, double current_lat, double current_lon, double target_lat, double target_lon, float tolerence, float kp, float lead_vx, float lead_vy ){
    
    const float R = 6378137.0;  // Earth radius in meters
    double dlat = (target_lat - current_lat) * M_PI / 180.0; //in radians
    double dlon = (target_lon - current_lon) * M_PI / 180.0; //in radians

    //normalize dlon to -pi to pi
    double dlon_normalized = fmod((dlon + M_PI), (2 * M_PI)) - M_PI;
    //debugging intermediate values
    double mean_lat = ((current_lat * M_PI/180) + (target_lat * M_PI/180)) / 2.0; //in radians

    //North-South component (y): R * delta_lat
    float y = R * dlat;

    //East-West component (x): R * delta_lon * cos(mean_lat)
    float x = R * dlon_normalized * cos(mean_lat);
    
    std::cout << "Distace to Target: "<< "X: " << x << " Y: " << y << std::endl;

    if (fabs(x) > tolerence) {
        float vx = kp * lead_vx;
        set_velocity(system, vx, 0.0, 0.0); // Send velocity command
        std::cout << "pos_x correction: " << vx << std::endl;
    }
    if (fabs(y) > tolerence) {
        float vy = kp * lead_vy;
        set_velocity(system, 0.0, vy, 0.0); // Send velocity command
        std::cout << "pos_y correction: " << vy << std::endl;
    }

}

bool set_flight_mode(System& system, uint8_t base_mode, uint8_t custom_mode) {
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        system.get_system_id(),
        base_mode,
        custom_mode
    );

    return mavlink_passthrough.send_message(msg) == MavlinkPassthrough::Result::Success;
}

bool enable_data_stream(System& system, uint8_t stream_id, uint16_t rate) {
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        system.get_system_id(),
        1, // target component
        stream_id,
        rate,
        1  // start streaming (1 = start, 0 = stop)
    );

    return mavlink_passthrough.send_message(msg) == MavlinkPassthrough::Result::Success;
}

double distance_to_home(System& system, Telemetry& telemetry) {
    // Get current position (non-blocking snapshot)
    Telemetry::Position position = telemetry.position();
    double current_lat = position.latitude_deg;
    double current_lon  = position.longitude_deg;
    std::cout << std::fixed << std::setprecision(7);
    std::cout << "Current position: " << current_lat << ", " << current_lon << std::endl;

    // Get home position
    Telemetry::Position home = telemetry.home();
    double home_lat = home.latitude_deg;
    double home_lon  = home.longitude_deg;

    return distance_between(current_lat, current_lon, home_lat, home_lon);
}

//main function

int main() {

    std::string rfd_address = "/dev/ttyUSB0"; // RFD900x serial port address
    Mavsdk::Configuration config{ComponentType::GroundStation};
    Mavsdk mavsdk(config);

    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    std::shared_ptr<System> system;
    while (true) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) {
            system = systems.at(0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Telemetry telemetry(*system);
    // telemetry.subscribe_health_all_ok([](bool all_ok) {
    //     std::cout << "Health status: " << (all_ok ? "OK" : "NOT OK") << std::endl;
    // });

    enable_data_stream(*system, MAV_DATA_STREAM_ALL,100);
    std::cout << "Data stream enabled." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Telemetry telemetry{system}; //initialize telemetry

    while (true) {  

        // get leader position

        // Telemetry::Position position = telemetry.position();

        // double current_lat = position.latitude_deg;
        // double current_lon = position.longitude_deg;
        // std::cout << "Current position: " << current_lat << ", " << current_lon << std::endl;

        // telemetry.subscribe_battery([](Telemetry::Battery battery) {
        //     std::cout << "Battery: " << static_cast<int32_t> (battery.remaining_percent) <<" %" << std::endl;
        // });

        // telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode) {
        //     std::cout << "Flight mode: " << flight_mode << std::endl;
        // });

        //std::atomic<bool> target_reached{false};
        //Telemetry::Position position = telemetry.position();
        std::vector<double> leader_pos = get_rfd900x_data(rfd_address);
        //leader data list: [leader_lat,leader_lon,leader_yaw,leader_vx,leader_vy,leader_vz,rng_distance]
        auto [lat, lon] = get_global_position(*system, telemetry);
        std::cout << std::fixed << std::setprecision(7);
        std::cout << "Global position: " "Current Lat:" << lat << " " << "Current Lon: "<<lon << std::endl;
        double dist_to_home = distance_to_home(*system, telemetry);
        std::cout << "Distance to home: " << dist_to_home << " meters." << std::endl;
        counter++;

        if (counter == 1) {
            // Set the flight mode to GUIDED
            set_flight_mode(*system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);
            std::cout << "Flight mode set to GUIDED." << std::endl;
            //arm and takeoff
            if (!takeoff(*system,form_alt)) {
                return 1;
            }
            std::cout << "Takeoff successful." << std::endl;
            auto [form_lat,form_lon] = relative_pos(leader_pos[0], leader_pos[1], dist_to_leader, leader_pos[2], angle_to_leader);
            formation(*system, angle_to_leader, dist_to_leader, form_alt, form_lat, form_lon, leader_pos[2]);
            std::chrono::seconds(1);
        
        }
        set_velocity(*system, leader_pos[3], leader_pos[4], 0.0f); 
        auto [form_lat,form_lon] = relative_pos(leader_pos[0], leader_pos[1], dist_to_leader, leader_pos[2], angle_to_leader);
        geo_distance_components(*system, lat, lon, form_lat, form_lon, pos_tolerence, kp, leader_pos[3], leader_pos[4]);

        //     std::cout << "Mission complete. Returning to home." << std::endl;
        //     set_flight_mode(*system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6);
        //     std::cout << "Flight mode set to RTL." << std::endl;
    }
}

//g++ follower.cpp -lmavsdk -lpthread -o follower (compile command)