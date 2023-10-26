#include "drone_controller.h"

int main(int argc, char **argv){


    ros::init(argc, argv, "drone_controller");
    ros::NodeHandle nh;
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    VixionAgent a(nh_ptr);
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}



