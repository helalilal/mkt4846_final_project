#include "Control.h"

namespace pid_controller
{
    Control::Control(ros::NodeHandle &t_node_handle)
        : m_node_handle(t_node_handle)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        m_right_lane_sub = m_node_handle.subscribe(m_right_lane_topic, 10, &Control::rightLaneCallback, this);

        m_left_lane_sub = m_node_handle.subscribe(m_left_lane_topic, 10, &Control::leftLaneCallback, this);

        m_park_sub = m_node_handle.subscribe(m_park_topic, 10, &Control::parkCallback, this);

        m_control_flags_sub = m_node_handle.subscribe(m_control_flags_topic, 10, &Control::controlFlagsCallback, this);

        m_vehicle_cmd_pub = m_node_handle.advertise<geometry_msgs::Twist>(m_vehicle_cmd_topic, 1000);

        m_target_pub = m_node_handle.advertise<visualization_msgs::Marker>("/target", 1000);

        ROS_INFO("Successfully launched node.");
    }

    bool Control::readParameters()
    {
        if (!m_node_handle.getParam("left_lane_topic", m_left_lane_topic))
        {
            return false;
        }
        if (!m_node_handle.getParam("right_lane_topic", m_right_lane_topic))
        {
            return false;
        }
        if (!m_node_handle.getParam("park_topic", m_park_topic))
        {
            return false;
        }
        if (!m_node_handle.getParam("vehicle_cmd_topic", m_vehicle_cmd_topic))
        {
            return false;
        }
        if (!m_node_handle.getParam("control_flags_topic", m_control_flags_topic))
        {
            return false;
        }
        if (!m_node_handle.getParam("car_lenght", m_L))
        {
            return false;
        }
        if (!m_node_handle.getParam("kp", m_kp))
        {
            return false;
        }
        if (!m_node_handle.getParam("kd", m_kd))
        {
            return false;
        }
        if (!m_node_handle.getParam("ki", m_ki))
        {
            return false;
        }
        if (!m_node_handle.getParam("velocity_limit", m_velocity_limit_lineer))
        {
            return false;
        }
        if (!m_node_handle.getParam("ke_stanley_param_turn", m_k_e_turn))
        {
            return false;
        }
        if (!m_node_handle.getParam("ke_stanley_param_steady", m_k_e_steady))
        {
            return false;
        }
        if (!m_node_handle.getParam("treshold", m_treshold))
        {
            return false;
        }
        if (!m_node_handle.getParam("target", m_target))
        {
            return false;
        }
        if (!m_node_handle.getParam("park_target", m_park_target))
        {
            return false;
        }
        if (!m_node_handle.getParam("min_steering_vehicle", m_min_steering_vehicle))
        {
            return false;
        }
        if (!m_node_handle.getParam("max_steering_vehicle", m_max_steering_vehicle))
        {
            return false;
        }
        return true;
    }

    void Control::controlFlagsCallback(const pid_controller::controlFlagsConstPtr &t_flags)
    {
        m_flags.path_choice = t_flags->path_choice;
        m_flags.controller_method = t_flags->controller_method;
        m_flags.stop_flag = t_flags->stop_flag;
        m_flags.blind_rush = t_flags->blind_rush;
        m_flags.park_mode = t_flags->park_mode;
        m_flags.is_autonomous = t_flags->is_autonomous;

        if (m_old_lane != m_flags.path_choice)
        {
            m_is_problem = false;
            m_old_path_right.clear();
            m_old_path_left.clear();
        }
        m_old_lane = m_flags.path_choice;
    }

    void Control::rightLaneCallback(const nav_msgs::PathConstPtr &t_path)
    {
        if (m_flags.path_choice != 0 || m_flags.park_mode == true)
        {
            m_old_path_right.clear();
            return;
        }

        std::vector<geometry_msgs::Point> temp_path;

        temp_path.resize(t_path->poses.size());
        for (long unsigned int i = 0; i < temp_path.size(); i++)
        {
            temp_path[i].x = t_path->poses[i].pose.position.x;
            temp_path[i].y = t_path->poses[i].pose.position.y;
            temp_path[i].z = t_path->poses[i].pose.position.z;
        }

        if (!m_old_path_right.empty() && !temp_path.empty())
        {
            geometry_msgs::Point point_new{temp_path[calculateTarget(temp_path.size())]};
            geometry_msgs::Point point_old{m_old_path_right[calculateTarget(m_old_path_right.size())]};

            float dist = getDistance(point_new.x, point_new.y, point_old.x, point_old.y);

            if (dist > m_treshold)
            {
                if (m_problem_counter != 10)
                {
                    temp_path = m_old_path_right;
                    m_is_problem = true;
                    m_problem_counter++;
                    ROS_WARN(" NOT WORKING");
                }
                else
                {
                    m_problem_counter = 0;
                    m_is_problem = false;
                }
            }
            else
            {
                m_is_problem = false;
                m_problem_counter = 0;
                ROS_ERROR("WORKING");
            }
        }

        startController(temp_path);

        if (!temp_path.empty() && !m_is_problem)
        {
            m_old_path_right = temp_path;
        }
    }

    void Control::leftLaneCallback(const nav_msgs::PathConstPtr &t_path)
    {
        if (m_flags.path_choice != 1 || m_flags.park_mode == true)
        {
            m_old_path_left.clear();
            return;
        }

        std::vector<geometry_msgs::Point> temp_path;

        temp_path.resize(t_path->poses.size());
        for (long unsigned int i = 0; i < temp_path.size(); i++)
        {
            temp_path[i].x = t_path->poses[i].pose.position.x;
            temp_path[i].y = t_path->poses[i].pose.position.y;
            temp_path[i].z = t_path->poses[i].pose.position.z;
        }

        if (!m_old_path_left.empty() && !temp_path.empty())
        {
            geometry_msgs::Point point_new{temp_path[calculateTarget(temp_path.size())]};
            geometry_msgs::Point point_old{m_old_path_left[calculateTarget(m_old_path_left.size())]};

            float dist = getDistance(point_new.x, point_new.y, point_old.x, point_old.y);

            if (dist > m_treshold)
            {
                if (m_problem_counter != 10)
                {
                    temp_path = m_old_path_left;
                    m_is_problem = true;
                    m_problem_counter++;
                    ROS_WARN("NOT WORKING");
                }
                else
                {
                    m_problem_counter = 0;
                    m_is_problem = false;
                }
            }
            else
            {
                m_problem_counter = 0;
                m_is_problem = false;
                ROS_ERROR("WORKING");
            }
        }

        startController(temp_path);

        if (!temp_path.empty() && !m_is_problem)
        {
            m_old_path_left = temp_path;
        }
    }

    void Control::parkCallback(const nav_msgs::PathConstPtr &t_path)
    {
        if (m_flags.park_mode == false)
        {
            return;
        }

        std::vector<geometry_msgs::Point> temp_path;

        temp_path.resize(t_path->poses.size());
        for (long unsigned int i = 0; i < temp_path.size(); i++)
        {
            temp_path[i].x = t_path->poses[i].pose.position.x;
            temp_path[i].y = t_path->poses[i].pose.position.y;
            temp_path[i].z = t_path->poses[i].pose.position.z;
        }

        startController(temp_path);
    }

    void Control::startController(const std::vector<geometry_msgs::Point> &t_path)
    {

        if (t_path.size() <= 0)
        {
            ROS_INFO("No path in selected option.");
            sendCommand((m_min_steering_vehicle+m_max_steering_vehicle)/2); // temp
            return;
        }

        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "velodyne";
        point_marker.header.stamp = ros::Time::now();
        point_marker.id = 0;
        point_marker.type = point_marker.SPHERE;
        point_marker.action = point_marker.ADD;
        point_marker.pose.position.x = t_path[calculateTarget(t_path.size())].x;
        point_marker.pose.position.y = t_path[calculateTarget(t_path.size())].y;
        point_marker.pose.position.z = 0.0;
        point_marker.pose.orientation.x = 0;
        point_marker.pose.orientation.y = 0;
        point_marker.pose.orientation.z = 0;
        point_marker.pose.orientation.w = 1;
        point_marker.scale.x = 0.8;
        point_marker.scale.y = 0.8;
        point_marker.scale.z = 0.8;
        point_marker.color.a = 1;
        point_marker.color.r = 1;
        point_marker.color.g = 0;
        point_marker.color.b = 0;
        point_marker.lifetime = ros::Duration(0.2);

        m_target_pub.publish(point_marker);

        if (m_flags.controller_method == "pid")
            pidController(t_path);
        else
            ROS_INFO("Control Flags are not received.");
    }

    void Control::pidController(const std::vector<geometry_msgs::Point> &t_path)
    {
        ROS_INFO("Running PID controller.");

        float goalPoint = t_path[calculateTarget(t_path.size())].y;

        error_pid = goalPoint - car_point;

        proportional = m_kp * error_pid;

        integrator = integrator + m_ki * (error_pid + prev_error_pid);

        /* Anti-wind-up via integrator clamping */
        if (integrator > limMaxInt)
        {

            integrator = limMaxInt;
        }
        else if (integrator < limMinInt)
        {

            integrator = limMinInt;
        }

        differentiator = m_kd * (error_pid - prev_error_pid);

        double steer = proportional + integrator + differentiator;

        if (steer > limMax)
        {

            steer = limMax;
        }
        else if (steer < limMin)
        {

            steer = limMin;
        }

        prev_error_pid = error_pid;

        prev_car_point = car_point;

        steer = normalizeForVehicle(steer, limMax, limMin, m_min_steering_vehicle, m_max_steering_vehicle);

        steer = numberRounder(steer);
        std::cout << " STEER FOR CAN -->> " << steer << std::endl;

        sendCommand(steer);
    }


    void Control::sendCommand(const float &steer)
    {
        if (!m_flags.is_autonomous)
        {
            return;
        }

        geometry_msgs::Twist cmdMsg;
        cmdMsg.linear.x = m_velocity_limit_lineer; // temp
        cmdMsg.angular.z = steer;                  // temp

        if (m_flags.stop_flag)
            cmdMsg.linear.x = 0; // temp

        if (m_flags.blind_rush)
            cmdMsg.angular.z = 0;

        m_vehicle_cmd_pub.publish(cmdMsg);
    }

    float Control::normalizeAngle(float angle)
    {
        if (angle > M_PI)
            angle -= 2 * M_PI;
        if (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    float Control::getDistance(float a1, float a2, float b1, float b2)
    {
        return std::sqrt((a1 - b1) * (a1 - b1) + (a2 - b2) * (a2 - b2));
    }

    int Control::calculateTarget(int t_path_size)
    {
        if (m_flags.park_mode)
        {
            return m_park_target;
        }

        if (t_path_size < m_target)
        {
            return t_path_size - 1;
        }
        return m_target;
    }

    double Control::normalizeForVehicle(const double input, const double min_current,
                                        const double max_current, const double min_target,
                                        const double max_target)
    {
        return (((input - min_current) / (max_current - min_current)) * (max_target - min_target)) + min_target;
    }

    double Control::numberRounder(const double &number)
    {
        return static_cast<int>(number / 10) * 10;
    }

} // namespace pid_controller