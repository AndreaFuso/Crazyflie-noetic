#include <ros/ros.h>
#include <crazyflie_messages/CustomTrajectoryPointYaw_srv.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <crazyflie_messages/Takeoff_srv.h>

class TrajectoryService {
    private:
        bool serviceCallback(crazyflie_messages::CustomTrajectoryPointYaw_srv::Request &req, crazyflie_messages::CustomTrajectoryPointYaw_srv::Response &res){
            ROS_INFO("SERVIZIO CHIAMATO! RICHIESTO PUNTO [%f %f %f] WITH YAW %f", req.trajectory_point.point.x, req.trajectory_point.point.y, req.trajectory_point.point.z, req.trajectory_point.yaw);
            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();

            // Default desired position and yaw.
            Eigen::Vector3d desired_position(req.trajectory_point.point.x, req.trajectory_point.point.y, req.trajectory_point.point.z);
            double desired_yaw = req.trajectory_point.yaw;

            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
              desired_position, desired_yaw, &trajectory_msg);

            trajectory_pub.publish(trajectory_msg);

            res.result = true;
            return true;
        }
    public:
        ros::Publisher trajectory_pub;
        ros::ServiceServer traj_service;

        TrajectoryService(ros::NodeHandle& nh){
            trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
            traj_service = nh.advertiseService("/go_to", &TrajectoryService::serviceCallback, this);
        }
};

class CrazyflieSimulatorManager {
    private:
        // TAKEOFF SERVICE CALLBACK
        bool takeoff_service_cb(crazyflie_messages::Takeoff_srv::Request &req, crazyflie_messages::Takeoff_srv::Response &res){
            ROS_INFO("[SERVICE CALLED] Takeoff service called...")

        }

    public:
        // Takeoff service declaration:
        ros::ServiceServer take_off_service;

        // CONSTRUCTOR
        CrazyflieSimulatorManager(ros::NodeHandle& nh){
            // Service instance:
            take_off_service = nh.advertiseService("/takeoff", &CrazyflieSimulatorManager::takeoff_service_cb, this);
        }
};





int main(int argc, char **argv) {

    // Node instantiation:
    ros::init(argc, argv, "crazyflie_sim_manager");
    ros::NodeHandle nh;
    TrajectoryService traj_service(nh);
    ros::spin();

    return 0;
}