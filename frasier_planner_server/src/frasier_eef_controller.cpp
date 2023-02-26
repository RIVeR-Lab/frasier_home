#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>

class FRASIEREEFController{
    public:
        FRASIEREEFController(ros::NodeHandle n);
        void threeDMouseCb(const geometry_msgs::Twist::ConstPtr &msg);

    private:
        ros::NodeHandle nh;
        geometry_msgs::Twist::ConstPtr twist_;
        geometry_msgs::Twist rounded_twist_;
};

FRASIEREEFController::FRASIEREEFController(ros::NodeHandle n){
    ros::Subscriber eef_twist_sub_ = nh.subscribe("/spacenav/twist", 1, &FRASIEREEFController::threeDMouseCb, this);
    ros::Publisher eef_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/hsrb/pseudo_endeffector_controller/command_velocity_with_base", 10);
    ros::topic::waitForMessage<geometry_msgs::Twist>("/spacenav/twist", ros::Duration(2));
    ros::Rate rate(100);
    geometry_msgs::TwistStamped cmd_twist;
    cmd_twist.header.frame_id = "base_link";
    while(ros::ok()){
        cmd_twist.twist.linear.x = rounded_twist_.linear.x * 0.2;
        cmd_twist.twist.linear.y = rounded_twist_.linear.y * 0.2;
        cmd_twist.twist.linear.z = rounded_twist_.linear.z * 0.2;

        cmd_twist.twist.angular.x = rounded_twist_.angular.x * 0.2;
        cmd_twist.twist.angular.y = rounded_twist_.angular.y * 0.2;
        cmd_twist.twist.angular.z = rounded_twist_.angular.z * 0.2;
        eef_vel_pub_.publish(cmd_twist);
        ros::spinOnce();
        rate.sleep();
    }

}

void FRASIEREEFController::threeDMouseCb(const geometry_msgs::Twist::ConstPtr &msg){
    twist_ = msg;
    
    rounded_twist_.linear.x = std::ceil(twist_->linear.x * 100.0) / 100.0;
    rounded_twist_.linear.y = std::ceil(twist_->linear.y * 100.0) / 100.0;
    rounded_twist_.linear.z = std::ceil(twist_->linear.z * 100.0) / 100.0;

    rounded_twist_.angular.x = std::ceil(twist_->angular.x * 100.0) / 100.0;
    rounded_twist_.angular.y = std::ceil(twist_->angular.y * 100.0) / 100.0;
    rounded_twist_.angular.z = std::ceil(twist_->angular.z * 100.0) / 100.0;

    std::cout << "rouneded twist linear: " <<
    rounded_twist_.linear.x << " " << 
    rounded_twist_.linear.y << " " << 
    rounded_twist_.linear.z << std::endl;

    std::cout << "rouneded twist angular: " <<
    rounded_twist_.angular.x << " " << 
    rounded_twist_.angular.y << " " << 
    rounded_twist_.angular.z << std::endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_eef_controller");
    ros::NodeHandle nh;

    FRASIEREEFController eef(nh);
}

