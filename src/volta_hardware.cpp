#include <volta_hardware/volta_hardware.h>
#include <boost/assign/list_of.hpp>

namespace volta_base {
void voltaHardware :: rpmCallback(const volta_msgs::RPM::ConstPtr& rpmTemp)
{
  this -> subMotorRPMRight	= rpmTemp->right;
  this -> subMotorRPMLeft	= rpmTemp->left;
}
voltaHardware::voltaHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq) {
    this->nh_ = nh;
    this->private_nh_ = private_nh_;
    this->target_control_freq = target_control_freq;
    private_nh_.param<double>("wheel_diameter", this->wheel_diameter_, 0.2);
    private_nh_.param<double>("max_speed", this->max_speed_, 1.0);
    this->max_rpm_ = MAX_RPM;

    timeFlag=0;
    prevRPMLeft=0;
    prevRPMRight=0;
    subMotorRPMRight = 0;
    subMotorRPMLeft = 0;

    pubMotorRPMRight=0;
    pubMotorRPMLeft=0;

    this->rpm_pub = nh.advertise<volta_msgs::RPM>("rpm_pub", 100);
    this->rpm_sub = nh.subscribe("rpm_sub", 100, &voltaHardware :: rpmCallback,this);
    this->register_controllers();
}

void voltaHardware::register_controllers() {
    ros::V_string joint_names = boost::assign::list_of("left_wheel_joint")
    ("right_wheel_joint");
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                          &joints_[i].position, &joints_[i].velocity,
                                                          &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
}

void voltaHardware::update_encoder_readings_to_joints() {
    double rpm_left, rpm_right;
    rpm_left 	=	(double)this -> subMotorRPMLeft;
    rpm_right 	= 	(double)this -> subMotorRPMRight;

    double left, right;
    left = this->convert_rpm_to_radians(rpm_left);
    right = this->convert_rpm_to_radians(rpm_right);

    for (int i=0; i<4; i++) {
        if (i%2 == 0) {
            this->joints_[i].velocity = left;
        } else {
            this->joints_[i].velocity = right;
        }
    }
}

void voltaHardware::send_velocity_to_motors_from_joints()
{
   volta_msgs :: RPM rpm;
   int16_t rpm_left, rpm_right;

   double left = this->joints_[0].velocity_command;
   double right = this->joints_[1].velocity_command;

   rpm_left = (int16_t)convert_radians_to_rpm(left);
   rpm_right = (int16_t)convert_radians_to_rpm(right);

   this->limit_speeds(rpm_left, rpm_right);

   rpm.right=rpm_right;
   rpm.left = rpm_left;

   this->rpm_pub.publish(rpm);

}

void voltaHardware::set_speeds(double left, double right) {
    int16_t rpm_left, rpm_right;

    rpm_left = (int16_t)convert_radians_to_rpm(left);
    rpm_right = (int16_t)convert_radians_to_rpm(right);

    this->limit_speeds(rpm_left, rpm_right);
}




void voltaHardware::limit_speeds(int16_t &left, int16_t &right) {
    int16_t temp_max = std::max(std::abs(left), std::abs(right));
    if (temp_max > this->max_rpm_) {
        left *= this->max_rpm_ / temp_max;
        right *= this->max_rpm_ / temp_max;
    }
}

double voltaHardware::convert_rpm_to_radians(double rpm) {
    return (double)((rpm*2.0*PI)/(60.0));
}

double voltaHardware::convert_radians_to_rpm(double radians) {
    double ret= (double)(radians*60.0)/(2.0*PI);
    return ret;
}
}
