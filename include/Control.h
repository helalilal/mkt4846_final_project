#pragma once

#include <ros/ros.h>

#include <vector>
#include <iostream>
#include <cmath>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <autoware_msgs/VehicleCmd.h>
#include <pid_controller/controlFlags.h>
#include <visualization_msgs/Marker.h>

namespace pid_controller
{
    class Control
    {
    public:
        /*!
         * Constructor.
         * @param t_node_handle the ROS node handle.
         */
        Control(ros::NodeHandle &t_node_handle);

    private:
        /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
        bool readParameters();

        /*!
         * ROS message filter callback method.
         * @param t_path the received Path message.
         */
        void rightLaneCallback(const nav_msgs::PathConstPtr &t_path);

        /*!
         * ROS message filter callback method.
         * @param t_path the received Path message.
         */
        void leftLaneCallback(const nav_msgs::PathConstPtr &t_path);

        /*!
         * ROS message filter callback method.
         * @param t_path the received Path message.
         */
        void parkCallback(const nav_msgs::PathConstPtr &t_path);

        /*!
         * ROS message filter callback method.
         * @param t_flags the received Flag message.
         */
        void controlFlagsCallback(const pid_controller::controlFlagsConstPtr &t_flags);

        /*!
         * Function to call the controller methods.
         */
        void startController(const std::vector<geometry_msgs::Point> &t_path);

        /*!
         * Pid controller.
         */
        void pidController(const std::vector<geometry_msgs::Point> &t_path);

        /*!
         * Function for sending commands to sim/car.
         */
        void sendCommand(const float & steer);

        /*!
         * Normalize angle function.
         * @return float normalized angle.
         */
        float normalizeAngle(float angle);

        /*!
         * Get distance function.
         * @return float distance.
         */
        float getDistance(float a1, float a2, float b1, float b2);

        /*!
         * Calculate target Waypoint.
         * @return int index.
         */
        int calculateTarget(int t_path_size);

        /*!
         * Scale a number from a range to different range.
         */
        double normalizeForVehicle(const double input, const double min_current,
                                   const double max_current, const double min_target,
                                   const double max_target);

        double numberRounder(const double &number);

        //! ROS node handle.
        ros::NodeHandle &m_node_handle;

        //! Right lane topic name.
        std::string m_right_lane_topic;

        //! Left lane topic name.
        std::string m_left_lane_topic;

        //! Park topic name.
        std::string m_park_topic;

        //! Vehicle Command topic name.
        std::string m_vehicle_cmd_topic;

        //! Control flags topic name.
        std::string m_control_flags_topic;

        //! Vehicle Command publisher.
        ros::Publisher m_vehicle_cmd_pub;

        //! Target publisher.
        ros::Publisher m_target_pub;

        //! Right lane subscriber.
        ros::Subscriber m_right_lane_sub;

        //! Left lane subscriber.
        ros::Subscriber m_left_lane_sub;

        //! Park subscriber.
        ros::Subscriber m_park_sub;

        //! Control flags subscriber.
        ros::Subscriber m_control_flags_sub;

        //! Controller method.
        std::string m_controller_method;

        //! Car lenght.
        double m_L;

        //! PID Parameter.
        double m_kp;

        //! PID Parameter.
        double m_kd;

        //! PID Parameter.
        double m_ki;

        //! PID Parameter.
        double m_steer_max;

        //! PID Parameter.
        double m_steer_min;

        // sample time (in seconds)
        double dt = 0.01f;

        // the vechicle works with local point
        double car_point = 0.00;
        double prev_car_point = 0.0; // needed for differentiator;

        /* Integrator limits */
        double limMinInt = -5.0;
        double limMaxInt = 5.0;

        /* Output limits */
        double limMin = -3.0;
        double limMax = 3.0;

        // controller memory
        double proportional = 0.0;
        double integrator = 0.0;
        double differentiator = 0.0;
        double error_pid = 0.0;
        double prev_error_pid = 0.0;

        //! Velocity limit.
        double m_velocity_limit_lineer;

        //! Flags.
        pid_controller::controlFlags m_flags;

        //! problem path flag.
        bool m_is_problem{false};

        //! Old lane flag.
        bool m_old_lane{false};

        //! problem counter.
        int m_problem_counter{0};

        //! k_e value for turns.
        float m_k_e_turn;

        //! k_e value for steady.
        float m_k_e_steady;

        //! Target waypoint threshold.
        int m_target;

        //! Park target waypoint threshold.
        int m_park_target;

        //! problem treshold.
        float m_treshold;

        //! Minimum value for steering
        double m_min_steering_vehicle;

        //! Maximum value for steering
        double m_max_steering_vehicle;

        //! Old right path.
        std::vector<geometry_msgs::Point> m_old_path_right;

        //! Old left path.
        std::vector<geometry_msgs::Point> m_old_path_left;
    };

} // namespace pid_controller
