#include <ros/ros.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
int main(int argc, char *argv[]) {
    ros::init(argc,argv,"can_serial_test");
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter double_param;
    dynamic_reconfigure::Config conf;
    double_param.name = "Base_Diag_EN_param";
    double_param.value = 10;
    conf.ints.push_back(double_param);
    double_param.name = "Base_Diag_Rate_param";
    double_param.value = 20;
    conf.ints.push_back(double_param);
    srv_req.config = conf;
    //srv_req.config = conf;
    ros::NodeHandle nh;
    int i=0;
    ros::Rate r(0.5);
    while (ros::ok()) {
        ROS_ERROR("%d", i++);
        if (ros::service::call("can_serial/set_parameters", srv_req, srv_resp)){
            ROS_ERROR("service call success");
        } else {
            ROS_ERROR("service call failed");
        }
        ros::spinOnce();
        r.sleep();
    }
}
