#include <ros/ros.h>
#include <CrazyflieBeta/CustomTrajectoryPointYaw_srv.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>



class TrajectoryService {
    private:
        bool serviceCallback(CrazyflieBeta::CustomTrajectoryPointYaw_srv::Request &req, CrazyflieBeta::CustomTrajectoryPointYaw_srv::Response &res){
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





int main(int argc, char **argv) {

    // Node instantiation:
    ros::init(argc, argv, "trajectory_handle_service_server");
    ros::NodeHandle nh;
    TrajectoryService traj_service(nh);



    ros::spin();

    return 0;
}