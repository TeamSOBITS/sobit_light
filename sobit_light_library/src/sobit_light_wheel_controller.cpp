#include "sobit_light_library/sobit_light_wheel_controller.hpp"


namespace sobit_light {

WheelController::WheelController ( const std::string &name )
: Node( name )
{
    if (!this->get_node_base_interface()) {
        throw std::runtime_error("Failed to initialize ROS2 Node.");
    }
     rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->shared_from_this());
    executor.spin_some();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&WheelController::callbackOdometry, this, std::placeholders::_1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mux/input/teleop", 1);

    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::seconds(3));
}

WheelController::WheelController ()
: Node("sobit_light_wheel_controller")
{
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&WheelController::callbackOdometry, this, std::placeholders::_1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mux/input/teleop", 1);

    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::seconds(3));
}

WheelController::WheelController(const rclcpp::NodeOptions & options)
: Node("sobit_light_wheel_controller", options) {
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&WheelController::callbackOdometry, this, std::placeholders::_1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mux/input/teleop", 1);
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::seconds(3));
}

bool WheelController::controlWheelLinear( const double distance ) {
    try {
        auto start_time = this->get_clock()->now();
        geometry_msgs::msg::Twist output_vel;

        while ( curt_odom_.pose.pose.position.x == 0 &&
                curt_odom_.pose.pose.position.y == 0 &&
                curt_odom_.pose.pose.position.z == 0 &&
                curt_odom_.pose.pose.orientation.w == 0 ) {

            rclcpp::spin_some(this->get_node_base_interface());
        }
        nav_msgs::msg::Odometry init_odom = curt_odom_;

        double moving_distance = 0.0;
        double target_distance = std::fabs( distance );

        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;

        double velocity_differential = Kp * distance;
        rclcpp::Rate loop_rate(20);

        while ( moving_distance < target_distance  ) {
            rclcpp::spin_some(this->get_node_base_interface());

            auto end_time = this->get_clock()->now();
            auto elapsed_time = (end_time - start_time).seconds();

            double vel_linear = 0.0;

            // PID
            if ( target_distance <= 0.1 ) {
                vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / 0.8 * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
            } else {
                vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / ( 8.0 / target_distance ) * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
            } 
            output_vel.linear.x = ( distance > 0 ) ? vel_linear : -vel_linear;
            velocity_differential = vel_linear;
            
            pub_cmd_vel_->publish( output_vel );

            double x_diif = curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x;
            double y_diif = curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y;
            moving_distance = std::hypot( x_diif, y_diif );

            RCLCPP_INFO(this->get_logger(), "x_diif = %f\tx2_diif = %f\ty_diif = %f\ty2_diif = %f", init_odom.pose.pose.position.x, curt_odom_.pose.pose.position.x, init_odom.pose.pose.position.y, curt_odom_.pose.pose.position.y );
            loop_rate.sleep();
        }

        return true;
    } catch ( const std::exception& ex ) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return false;
    }
}

bool WheelController::controlWheelRotateRad( const double angle_rad ) {
    try {
        while ( curt_odom_.pose.pose.position.x == 0 &&
                curt_odom_.pose.pose.position.y == 0 &&
                curt_odom_.pose.pose.position.z == 0 &&
                curt_odom_.pose.pose.orientation.w == 0 ) {

            rclcpp::spin_some(this->get_node_base_interface());
        }

        auto start_time = this->get_clock()->now();

        geometry_msgs::msg::Twist output_vel;
        double init_yaw = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
        double moving_angle_rad = 0.0;
        double abs_angle_rad =  std::fabs( angle_rad );
        double abs_angle_deg = rad2Deg( abs_angle_rad );

        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;

        double velocity_differential = Kp * angle_rad;
        int loop_cnt = 1;
        rclcpp::Rate loop_rate(20);

        while ( moving_angle_rad < abs_angle_rad ) {
            rclcpp::spin_some(this->get_node_base_interface());

            auto end_time = this->get_clock()->now();
            auto elapsed_time = (end_time - start_time).seconds();
            double vel_angular = 0.0;

            // PID
            if ( abs_angle_deg <= 30 ) {
                vel_angular = Kp * ( abs_angle_rad + 0.001 - moving_angle_rad )
                            - Kd * velocity_differential
                            + Ki * ( abs_angle_rad + 0.001 - moving_angle_rad ) * std::pow( elapsed_time, 2 );
            } else {
                vel_angular = Kp * ( abs_angle_rad + 0.001 - moving_angle_rad )
                            - Kd * velocity_differential
                            + Ki * ( abs_angle_rad + 0.001 - moving_angle_rad ) * std::pow( elapsed_time, 2 ) * 0.75 * 30 / abs_angle_deg;
            }
            output_vel.angular.z = ( angle_rad > 0 ) ? vel_angular : - vel_angular;
            velocity_differential = vel_angular;
            
            pub_cmd_vel_->publish( output_vel );
            
            double curt_yaw = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
            double pre_move_ang_rad = moving_angle_rad;

            if ( -0.00314 < curt_yaw - init_yaw && curt_yaw - init_yaw < 0 && 0 < angle_rad ) continue;
            else if ( 0 < curt_yaw - init_yaw && curt_yaw - init_yaw < 0.00314 && angle_rad < 0 ) continue;

            if ( curt_yaw - init_yaw < 0 && 0 < angle_rad ) moving_angle_rad = std::abs(curt_yaw - init_yaw + deg2Rad(360 * loop_cnt));
            else if ( 0 < curt_yaw - init_yaw && angle_rad < 0 ) moving_angle_rad = std::abs(curt_yaw - init_yaw - deg2Rad(360 * loop_cnt));
            else if ( 0 < angle_rad ) moving_angle_rad = std::abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
            else moving_angle_rad = std::abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));

            if ( rad2Deg(moving_angle_rad) < (rad2Deg(pre_move_ang_rad)-0.0314) ) {
                loop_cnt++;
                if ( 0 < angle_rad ) moving_angle_rad = std::abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
                else moving_angle_rad = std::abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));
            }
            
            RCLCPP_INFO(this->get_logger(), "target_angle = %f\tmoving_angle = %f", abs_angle_rad, moving_angle_rad );
            
            loop_rate.sleep();
        }
        return true;
    } catch ( const std::exception& ex ) {
        RCLCPP_ERROR( this->get_logger(), "%s", ex.what() );
        return false;
    }
}

bool WheelController::controlWheelRotateDeg( const double angle_deg ) {
    return controlWheelRotateRad( deg2Rad(angle_deg) );
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::WheelController)