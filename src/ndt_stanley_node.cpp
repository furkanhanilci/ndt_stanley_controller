#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <math.h>
#include <vector>
#include "../include/Linear_Interpolation.h" // fh
#include "../include/BicycleModel.h" // fh
#include "../include/StanleyController.h" // fh
#include <nav_msgs/Path.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std;
using namespace message_filters;

std::vector<double> global_x;
std::vector<double> global_y;

std::vector<double> state_x;
std::vector<double> state_y;


message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_robot_sub; // reference path
message_filters::Subscriber<nav_msgs::Path> *pose_wh_sub; // current state


typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Path> MySyncPolicy;
Synchronizer<MySyncPolicy> *message_sync;

void callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const nav_msgs::Path::ConstPtr& path_msg) {

    ROS_INFO("Reference Pose: [x: %f, y: %f, z: %f]", pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
    double ref_global_x = pose_msg->pose.position.x;
    double ref_global_y = pose_msg->pose.position.y;
    global_x.push_back(pose_msg->pose.position.x);
    global_y.push_back(pose_msg->pose.position.y);

    if (!path_msg->poses.empty()) {
        const auto& last_pose = path_msg->poses.back().pose.position;
        ROS_INFO("State Pose: [x: %f, y: %f]", last_pose.x, last_pose.y);
        double ref_state_x = last_pose.x;
        double ref_state_y = last_pose.y;
        state_x.push_back(last_pose.x);
        state_y.push_back(last_pose.y);


        double diff_x = ref_global_x - ref_state_x;
        double diff_y = ref_global_y - ref_state_y;
        double distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
        ROS_INFO("Difference - X: %f, Y: %f, Distance: %f", diff_x, diff_y, distance);
    }
}

void plot_waypoints_interpolation( const std::vector<Eigen::VectorXd> waypoints){
    std::vector<double> x_vals, y_vals;
    for (const auto& waypoint : waypoints) {
        x_vals.push_back(waypoint[0]);
        y_vals.push_back(waypoint[1]);
    }
}


double mapSteeringAngle(double angle) {

    double constrainedAngle = std::max(-12.0, std::min(12.0, angle));
    double mappedAngle = (constrainedAngle + 12) * (700 + 700) / (12 + 12) - 700;
    return mappedAngle;
}


void ComputeControl(const vector<Eigen::VectorXd> waypoints, const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp) {
    BicycleModel vehicle(0.0, 0.0, 0.0, 0.0);
    StanleyController controller(waypoints);

    vector<double> vehicle_x, vehicle_y, vehicle_theta;

    if (global_x.size() < 2 || global_y.size() < 2 || state_x.size() < 2 || state_y.size() < 2) {
        ROS_WARN("Not enough waypoints to compute control.");
        return;
    }

    for (size_t i = 0; i < global_x.size(); ++i) {

        double rawSteeringAngle = controller.GetDelta();
        double mappedSteeringAngle = mapSteeringAngle(rawSteeringAngle);

        controller.findClosestWaypoint(state_x[i], state_y[i], wp_distance, wp_interp_hash, wp_interp);
        double velocity = vehicle.getV();

        vehicle_x.push_back(state_x[i]);
        vehicle_y.push_back(state_y[i]);
        vehicle_theta.push_back(vehicle.getTheta());

        size_t closestIndex = controller.getClosestIndex();
        vector<Eigen::VectorXd> new_waypoints = controller.getNewWaypoints();
        controller.computeCrossTrackError(state_x[i], state_y[i], vehicle.getYaw());
        double target_idx = controller.GetTargetIdx();

        double x_target = new_waypoints[target_idx](0);
        double y_target = new_waypoints[target_idx](1);

        controller.computePID(10.0 , velocity); // Target speed in m/s
        controller.computeSteeringAngle(vehicle.getYaw(), velocity);
        cout << "velocity: " << velocity << " km/h" << endl;

        vehicle.update(mappedSteeringAngle, 0.01, 0.1, controller.GetMaxSteer());
        float steer_degree = mappedSteeringAngle ;
        cout << "Mapped steering angle: " << steer_degree << " degrees" << endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_subscriber_synchronized");
    ros::NodeHandle nh;

    pose_robot_sub = new Subscriber<geometry_msgs::PoseStamped>(nh, "/current_pose", 10);
    pose_wh_sub = new Subscriber<nav_msgs::Path>(nh, "/robot_path", 10);

    message_sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *pose_robot_sub, *pose_wh_sub);
    message_sync->registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        Linear_Interpolation linear_interpolation(global_x, global_y, 0.0001);
        Linear_Interpolation interp(global_x, global_y);
        std::vector<Eigen::VectorXd> waypoints = linear_interpolation.getWaypoints();
        interp.interpolateWaypoints();
        vector<Eigen::VectorXd> wp_interp = interp.getWp_interp();
        vector<int> wp_interp_hash = interp.getWp_interp_hash();
        vector<double> wp_distance = interp.getWp_distance();
        ComputeControl(waypoints, wp_distance, wp_interp_hash, wp_interp);
        ros::spinOnce();
        loop_rate.sleep();

    }

    delete pose_robot_sub;
    delete pose_wh_sub;
    delete message_sync;

    return 0;
}