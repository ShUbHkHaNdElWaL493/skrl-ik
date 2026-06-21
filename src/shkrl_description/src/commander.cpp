/*
    CS22B1090
    Shubh Khandelwal
*/

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shkrl_msgs/srv/get_tcp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CommanderNode : public rclcpp::Node
{

    private:

    double dt;
    rclcpp::Service<shkrl_msgs::srv::GetTCP>::SharedPtr get_tcp_service;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joints_client;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void get_tcp_callback(const std::shared_ptr<shkrl_msgs::srv::GetTCP::Request> request, std::shared_ptr<shkrl_msgs::srv::GetTCP::Response> response)
    {

        response->success = false;
        response->tcp[0] = 0.0;
        response->tcp[1] = 0.0;
        response->tcp[2] = 0.0;
        response->tcp[3] = 0.0;
        response->tcp[4] = 0.0;
        response->tcp[5] = 0.0;

        control_msgs::action::FollowJointTrajectory::Goal goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = request->joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = request->joints;
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = dt * 1e9;
        goal_msg.trajectory.points.push_back(point);

        auto goal_handle_future = this->joints_client->async_send_goal(goal_msg);
        goal_handle_future.wait();
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
            return;
        }

        auto result_future = this->joints_client->async_get_result(goal_handle);
        result_future.wait();
        auto result = result_future.get();
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal cancelled.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                request->target_frame,
                request->source_frame,
                tf2::TimePointZero,
                std::chrono::nanoseconds(static_cast<size_t>(dt * 1e9))
            );
            response->success = true;
            response->tcp[0] = t.transform.translation.x;
            response->tcp[1] = t.transform.translation.y;
            response->tcp[2] = t.transform.translation.z;
            response->tcp[3] = t.transform.translation.x;
            response->tcp[4] = t.transform.translation.y;
            response->tcp[5] = t.transform.translation.z;
        } catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Lookup Error: %s", ex.what());
        }

    }

    public:

    CommanderNode() :
    Node("commander_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    tf_buffer(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer))
    {

        this->get_parameter("frequency", dt);
        dt = 1.0 / dt;

        std::string joint_controller;
        this->get_parameter("joint_controller", joint_controller);
        this->joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, joint_controller + "/follow_joint_trajectory");

        this->get_tcp_service = this->create_service<shkrl_msgs::srv::GetTCP>(
            "/get_tcp",
            std::bind(&CommanderNode::get_tcp_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for JTC action server...");
        this->joints_client->wait_for_action_server();

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());
    rclcpp::shutdown();
    return 0;
}