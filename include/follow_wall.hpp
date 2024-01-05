#include "ros/ros.h"

class Robot {
    public:
     Robot(std::vector<std::pair<int, int>> sectors = {{330, 30}, {30, 60}, {75, 105}, {240, 300}});
     void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     void actionCallback(const std_msgs::String::ConstPtr& action);
     float radToDegrees(float radians);

     int actionLookup();
     void executeAction(ros::Duration duration, ros::Publisher pub);

     void moveForward();
     void moveLeft();
     void moveRight();
     [[nodiscard]] inline float& getStateRightFront() { return state_distance_["state-front"]; }
     [[nodiscard]] inline float& getStateRight() { return state_distance_["state-right-front"]; }       
     [[nodiscard]] inline float& getStateFront() { return state_distance_["state-right"]; }
     [[nodiscard]] inline float& getStateLeft() { return state_distance_["state-left"]; }
     [[nodiscard]] inline const uint16_t& getStateOrientation() const { return state_orientation_; }

     follow_wall::Diff diff_msg;
    private:
     //ros::NodeHandle node_handle_;
     uint16_t state_orientation_;
     std::map<std::string, float> state_distance_;
     const std::vector<std::pair<int, int>> sectors_;
};