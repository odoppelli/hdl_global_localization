#include <ros/ros.h>
#include <hdl_global_localization/SetRansacBox.h>


bool set_box(hdl_global_localization::SetRansacBox::Request &req, hdl_global_localization::SetRansacBox::Response &res){
    /*
    todo: set minimum values for the _span parameters and check if they are met
    x_span >= 0.5
    y_span >= 0.5
    z_span >= 0.25
    */
    if((req.x_span<=0.5) || (req.y_span<=0.5)){
        ROS_ERROR_STREAM("Service did not execute: x- or y_span too small. Must be >=0.5");
        return false;
    }
    if (req.z_span<=0.25){
        ROS_ERROR_STREAM("Service did not run: z_span too small. Must be >=0.25");
        return false;
    }
    
    double x_min, x_max, y_min, y_max, z_min, z_max;
    x_min = req.x_mid - req.x_span/2;
    x_max = req.x_mid + req.x_span/2;
    y_min = req.y_mid - req.y_span/2;
    y_max = req.y_mid + req.y_span/2;
    z_min = req.z_mid - req.z_span/2;
    z_max = req.z_mid + req.z_span/2;

    ros::param::set("/hdl_global_localization/ransac/x_min", x_min);
    ros::param::set("/hdl_global_localization/ransac/x_max", x_max);
    ros::param::set("/hdl_global_localization/ransac/y_min", y_min);
    ros::param::set("/hdl_global_localization/ransac/y_max", y_max);
    ros::param::set("/hdl_global_localization/ransac/z_min", z_min);
    ros::param::set("/hdl_global_localization/ransac/z_max", z_max);
    
    ROS_INFO_STREAM("Ransac Box set successfully");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "set_ransac_box_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/hdl_global_localization/set_ransac_box", set_box);
    ROS_INFO_STREAM("Ready to set RANSAC bounding box");
    ros::spin();

    return 0;
}