#include "follow_wall/Diff.h"
#include "follow_wall/State.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <limits>
#include <memory>
#include <sstream>

#include "follow_wall.hpp"

Robot::Robot(std::vector<std::pair<int, int>> sectors) : sectors_(sectors) {
    this->state_distance_.insert({"state-front", std::numeric_limits<float>::infinity()});
    this->state_distance_.insert({"state-right-front", std::numeric_limits<float>::infinity()});
    this->state_distance_.insert({"state-right", std::numeric_limits<float>::infinity()});
    this->state_distance_.insert({"state-left", std::numeric_limits<float>::infinity()}); 
    this->state_orientation_ = std::numeric_limits<uint16_t>::infinity(); 
}

float Robot::radToDegrees(float radians) {
    return ((radians)*180./M_PI);
} 

void Robot::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    // Total number of range mesurements 
    int count = scan->scan_time / scan->time_increment;

    // Orientation is the angle of minimum range 
    this->state_orientation_ = std::round((360.0f / count) * std::distance(std::begin(scan->ranges), 
                      std::min_element(std::begin(scan->ranges), std::end(scan->ranges))));

    auto state_it = this->state_distance_.begin();
    auto sector_it = this->sectors_.begin();
    for (; sector_it != this->sectors_.end(); state_it++, sector_it++) {
        int sector_index = sector_it - this->sectors_.begin(); 
        int begin = std::round(sector_it->first / 360.0f * count);
        int end = std::round(sector_it->second / 360.0f * count);
        if (sector_index == 0) { // front sector is non-continuous
            std::vector<float> sector_first(scan->ranges.begin() + begin, scan->ranges.end());
            std::vector<float> sector_second(scan->ranges.begin(), scan->ranges.begin() + end);
            sector_first.insert(sector_first.end(), sector_second.begin(), sector_second.end());
            float min = *min_element(sector_first.begin(), sector_first.end());
            if (isinf(min)) { min = 100.0; }
            this->state_distance_["state-front"] = min; 
        } else {
            std::vector<float> sector_ranges(scan->ranges.begin() + begin, scan->ranges.begin() + end);
            float min = *min_element(sector_ranges.begin(), sector_ranges.end());
             if (isinf(min)) { min = 100.0; }
            state_it->second = min;
        }
    }

    ROS_INFO("State: %f, %f, %f, %f, %d", this->state_distance_["state-front"],
                                          this->state_distance_["state-right-front"],
                                          this->state_distance_["state-right"],
                                          this->state_distance_["state-left"],
                                          this->state_orientation_);
}


void Robot::actionCallback(const std_msgs::String::ConstPtr& action) {
    
    this->diff_msg.left_direction_forward = true;
    this->diff_msg.right_direction_forward = true;
    
    if (action->data.c_str() == std::string("forward")) {
        this->diff_msg.left_speed = 100;
        this->diff_msg.right_speed = 100;
    } else if (action->data.c_str() == std::string("left")) {
        this->diff_msg.left_speed = 50;
        this->diff_msg.right_speed = 255;
    } else if (action->data.c_str() == std::string("right")) {
        this->diff_msg.left_speed = 255;
        this->diff_msg.right_speed = 50;
    } else {
        this->diff_msg.left_speed = 0;
        this->diff_msg.right_speed = 0;
    }  

}

int main (int argc, char **argv) {

    ros::init(argc, argv, "follow_wall_node");
    ros::NodeHandle node_handle;
    
    std::unique_ptr<Robot> robot = std::make_unique<Robot>();

    // Subscribe to laser scan topic
    ros::Subscriber scan_sub = node_handle.subscribe("/scan", 1000, &Robot::scanCallback, robot.get());

    // Subscribe to action topic
    ros::Subscriber action_sub = node_handle.subscribe("/action", 1000, &Robot::actionCallback, robot.get());
    
    // Robot state publisher
    ros::Publisher state_pub = node_handle.advertise<follow_wall::State>("robot_state", 1000);

    // Robot motor publisher
    ros::Publisher motor_pub = node_handle.advertise<follow_wall::Diff>("drive_motors", 1000);
        
    ros::Rate loop_rate(20); // loop rate is 20 hz

    ros::Duration timestep_duration = ros::Duration(0.1); // timestep is 0.5 seconds

    follow_wall::State state_msg;
    while (ros::ok()) {
        state_msg.front_distance = robot->getStateFront();
        state_msg.right_front_distance = robot->getStateRightFront();
        state_msg.right_distance = robot->getStateRight();
        state_msg.left_distance = robot->getStateLeft();
        state_msg.orientation = robot->getStateOrientation();
    
        state_pub.publish(state_msg);

        ros::Time future_timestep = ros::Time::now() + timestep_duration;
        motor_pub.publish(robot->diff_msg);
        while (ros::Time::now() < future_timestep) { }
        
        // Publish speed zero here
        robot->diff_msg.left_speed = 0;
        robot->diff_msg.right_speed = 0;
        motor_pub.publish(robot->diff_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
