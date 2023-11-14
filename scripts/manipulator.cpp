#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <math.h>
#include <string>
#include <vector>
#include "xarm/wrapper/xarm_api.h"

/**
 * Move the xArm using API calls 
 */

class Manipulator {
private:
    XArmAPI arm;
    float pregrasp_offset;
    float ee_length_offset;
    std::vector<float> init_pose;
    std::vector<float> orientation;
    std::vector<std::vector<float>> to_basket_points;
    std::vector<std::vector<float>> from_basket_points;
    bool encore;
    std::string ip;

public:
    Manipulator() {
        // Initialize values
        ros::NodeHandle nh;
        ros::param::get("/xarm_robot_ip", ip);
        ros::param::get("/encore", encore);

        std::string arm_yaml;
        ros::param::get("/arm_yaml", arm_yaml);
        parseYaml(arm_yaml);

        // Initialize xArm
        arm = XArmAPI(ip);
        arm.motion_enable(true);
        arm.connect();
        arm.set_mode(0);
        arm.set_state(0);
    }

    ~Manipulator() {
        arm->disconnect();
        delete arm;
    }

    void parseYaml(const std::string &yaml_file) {
        // Load and parse the YAML file
        YAML::Node config = YAML::LoadFile(yaml_file);
        pregrasp_offset = config["pregrasp_offset"].as<float>();
        ee_length_offset = config["ee_offset"].as<float>();
        init_pose = config["init_pose"].as<std::vector<float>>();
        orientation = config["orientation"].as<std::vector<float>>();
        to_basket_points = config["to_basket_points"].as<std::vector<std::vector<float>>>();
        from_basket_points = config["from_basket_points"].as<std::vector<std::vector<float>>>();
    }

    void moveToInit() {
        // Move to initial position
        ROS_INFO("Moving to initial pose");
        arm.set_position(init_pose, -1, 30, 0, 0, true, NO_TIMEOUT, false, 0);
        // int set_position(float pose[6], float radius=-1, float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, bool relative = false, unsigned char motion_type=0)
    }

    void cartesianMove(double dist, int axis) {
        dist *= 1000; // Convert m to mm
        std::vector<float> current_pos = arm.position;
        current_pos[axis] += dist; // Add dist to specified axis
        std::cout << "Executing cartesian move" << std::endl;
        arm->set_position(pose=current_pos, wait=true, speed=10);
    }

    void moveToPregrasp(geometry_msgs::Point poi_pose) {
        // Get the position and orientation
        double x = poi_pose.position.x;
        double y = poi_pose.position.y;
        double z = poi_pose.position.z;

        // Add x offsets
        x -= pregrasp_offset;
        x -= ee_length_offset;

        // Move to new position
        arm->set_position({x * 1000, y * 1000, z * 1000}, orientation, true, 30);

        // Update roll angle
        if (encore) {
            // Convert quaternion to rotation matrix
            Eigen::Quaterniond quaternion(quat.w, quat.x, quat.y, quat.z);
            Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

            // Extract the y and z components
            double y_comp = rotation_matrix(1, 0);
            double z_comp = rotation_matrix(2, 0);

            // Calculate the roll angle
            double theta = std::atan2(y_comp, z_comp);
            orientation[1] += (M_PI - theta) * (180.0 / M_PI);

            arm->set_position({x * 1000, y * 1000, z * 1000}, orientation, true, 30);
        }
    }

    void moveToPoi() {
        cartesianMove(pregrasp_offset, 0); // Move forward in x
    }

    void orientParallel() {
        std::vector<float> current_pos = arm.position;
        float pose[6] = [current_pos[0], current_pos[1], current_pos[2], orientation[0], orientation[1], orientation[2]];
        arm->set_position({current_pos[0], current_pos[1], current_pos[2]}, orientation, true, 30);
    }

    void moveToBasket() {
        cartesianMove(-0.05, 0); // Move back 5 cm
        orientParallel(); // Straighten orientation if needed
        cartesianMove(-0.18, 0); // Move back 18 cm
        execute_traj(to_basket_points);
        ROS_WARN("Executed traj to basket");
    }

    void moveFromBasket() {
        execute_traj(from_basket_points);
        ROS_WARN("Executed traj from basket");
    }

    void multiframe() {
        std::cout << "Multiframe: scanning down the plant" << std::endl;
        cartesianMove(-0.2, 2); // Move up 20 cm in z
    }

    void execute_traj(const std::vector<std::vector<float>>& points) {
        // Execute an interpolated trajectory of waypoints
        arm->move_arc_lines(points, 50, 1, true);
    }

    void disconnect() {
        // disconnect from xArm
        arm->disconnect();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "manipulator_node");
    Manipulator manipulator;

    ros::spin();

    return 0;
}