/*
 * scout_key_controller_node.cpp
 *
 * Created on: April 29, 2020 
 * Description:
 *
 * Copyright (c) 2020 Xianyu Qi
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <thread>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_ENTER 0x0a

int kfd = 0;
//机器人允许的最大速度
//  scout min parameters
const double scout_mini_linear_max = 2.5;    //m/s
const double scout_mini_angular_max = 50.5235; // rad/s

struct termios cooked, raw;

class ScoutKeyController
{
public:
        ScoutKeyController();
        ~ScoutKeyController()
        {
                tcsetattr(kfd, TCSANOW, &cooked);
        }
        void keyLoop();
        void publishVelocityLoop();

private:
        inline double deg2rad(double degree)
        {
                return degree * 0.01745329251994329576;
        }

        inline double rad2deg(double rad)
        {
                return rad * 57.29577951308232087721;
        }

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        double linear_vel_max_permit, angular_vel_max_permit;
        ros::Publisher twist_pub_;
        geometry_msgs::Twist twist_;
        double linear_vel_step_;
        double angular_vel_step_;
        std::thread *p_read_key_thread_;
        bool quit_requested;
};

ScoutKeyController::ScoutKeyController() : private_nh_("~"),
                                           quit_requested(false)

{
        // 最大线速度  [m/s]
        if (!private_nh_.getParam("linear_vel_max", linear_vel_max_permit))
        {
            linear_vel_max_permit = scout_mini_linear_max;
        }
        else
        {
                if (linear_vel_max_permit > scout_mini_linear_max)
                    linear_vel_max_permit = scout_mini_linear_max;
        }

        // 最大角速度  [rad/s]
        if (!private_nh_.getParam("angular_vel_max", angular_vel_max_permit))
        {
            angular_vel_max_permit = scout_mini_angular_max;
        }
        else
        {
                if (deg2rad(angular_vel_max_permit) > scout_mini_angular_max)
                    angular_vel_max_permit = scout_mini_angular_max;
                else
                {
                    angular_vel_max_permit = deg2rad(angular_vel_max_permit);
                }
        }

        // 线速度增量  [m/s]
        if (!private_nh_.getParam("linear_vel_step", linear_vel_step_))
                linear_vel_step_ = 0.12;

        // 角速度增量  [rad/s]
        if (!private_nh_.getParam("angular_vel_step", angular_vel_step_))
        {
                angular_vel_step_ = deg2rad(40);
        }
        else
        {
                angular_vel_step_ = deg2rad(angular_vel_step_);
        }

        twist_.angular.z = 0;
        twist_.linear.x = 0;
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
        p_read_key_thread_ = new std::thread(&ScoutKeyController::keyLoop, this);
}
void quit(int sig)
{
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
}

void ScoutKeyController::keyLoop()
{
        char c;
        while (true)
        {

                if (read(kfd, &c, 1) < 0)
                {
                        perror("read():");
                        exit(-1);
                        ROS_INFO_STREAM("error");
                }

                switch (c)
                {
                case KEYCODE_L:
                {
                        double command_angular_vel = twist_.angular.z + angular_vel_step_;
                        if (command_angular_vel >= angular_vel_max_permit)
                        {
                                twist_.angular.z = angular_vel_max_permit;
                        }
                        else
                        {
                                twist_.angular.z = command_angular_vel;
                        }
                        ROS_INFO_STREAM("ScoutKeyController: angular velocity incremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;
                }

                case KEYCODE_R:
                {
                        double command_angular_vel = twist_.angular.z - angular_vel_step_;

                        if (command_angular_vel <= -angular_vel_max_permit)
                        {
                                twist_.angular.z = -angular_vel_max_permit;
                        }
                        else
                        {
                                twist_.angular.z = command_angular_vel;
                        }
                        ROS_INFO_STREAM("ScoutKeyController: angular velocity decremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;
                }

                case KEYCODE_U:

                {
                        double command_linear_vel = twist_.linear.x + linear_vel_step_;

                        if (command_linear_vel >= linear_vel_max_permit)
                        {
                                twist_.linear.x = linear_vel_max_permit;
                        }
                        else
                        {
                                twist_.linear.x = command_linear_vel;
                        }
                        ROS_INFO_STREAM("ScoutKeyController: linear velocity incremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;
                }

                case KEYCODE_D:
                {
                        double command_linear_vel = twist_.linear.x - linear_vel_step_;

                        if (command_linear_vel <= -linear_vel_max_permit)
                        {
                                twist_.linear.x = -linear_vel_max_permit;
                        }
                        else
                        {
                                twist_.linear.x = command_linear_vel;


                        }
                        ROS_INFO_STREAM("ScoutKeyController: angular velocity decremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;
                }

                case KEYCODE_SPACE:
                        ROS_DEBUG("STOP");
                        twist_.linear.x = 0;
                        twist_.angular.z = 0;
                        ROS_INFO_STREAM("ScoutKeyController: linear and angular velocity decremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;

                case KEYCODE_ENTER:
                        ROS_DEBUG("STOP");
                        twist_.linear.x = 0;
                        twist_.angular.z = 0;
                        ROS_INFO_STREAM("ScoutKeyController: linear and angular velocity decremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
                        break;

                case KEYCODE_Q:
                        ROS_DEBUG("QUIT");
                        ROS_INFO_STREAM("Quit the ScoutKeyController successfully");
                        quit_requested = true;
                        return;
                        break;
//                    default:
//                       // ROS_DEBUG("STOP");
//                        twist_.linear.x = 0;
//                        twist_.angular.z = 0;
//                       // ROS_INFO_STREAM("ScoutKeyController: linear and angular velocity decremented [" << twist_.linear.x << " m/s | " << rad2deg(twist_.angular.z) << " deg/s ]");
//                        break;
                }
        }
}
void ScoutKeyController::publishVelocityLoop()
{
        ros::Rate rh(100); // hz
        while (!quit_requested && ros::ok())
        {
                twist_pub_.publish(twist_);
                ros::spinOnce();
                rh.sleep();
        }
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        twist_pub_.publish(twist_);
        p_read_key_thread_->join();
        delete p_read_key_thread_;
        p_read_key_thread_ = NULL;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "scout_key_controller");
        ros::NodeHandle nh;

        signal(SIGINT, quit);
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);

        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
        puts("---------------------------");
        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys to move the robot.");
        puts("Press the space bar to stop the robot.");
        puts("Press q to stop the program");

        ScoutKeyController scout_key_controller;
        scout_key_controller.publishVelocityLoop();
        return (0);
}
