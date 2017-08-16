#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <Biclops.h>
#include <biclops_controller/biclops_command.h>

using namespace std;
using namespace ros;

#define A 97
#define PI 3.14159265

// Defines which axes we want to use.
int axisMask = Biclops::PanMask + Biclops::TiltMask;

// Pointers to each axis (populated once controller is initialized).
PMDAxisControl *pan_axis = NULL;
PMDAxisControl *tilt_axis = NULL;

// biclops interface
Biclops biclops;

double pan_pos_value  = 0;
double tilt_pos_value = 0;
double pan_joint_pos  = 0;
double tilt_joint_pos = 0;

PMDAxisControl::Profile panProfile,tiltProfile;

// TODO get rid of quaternions, switch to pan/tilt (kinect driver will wait till pan/tilt is right)
// TODO publish current state
// TODO initialize neutral to baxter


bool tuck_biclops(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    ROS_INFO("[biclops_controller]: tucking the biclops.");
    biclops.HomeAxes(axisMask,true);

    pan_pos_value  = 0;
    tilt_pos_value = 0;
    pan_joint_pos  = 0;
    tilt_joint_pos = 0;

    return true;
}


void tip_orientation_callback(biclops_controller::biclops_command msg){
    //TODO add offset

    double panrev  = (msg.pan / (2 * PI));
    double tiltrev = (msg.tilt / (2 * PI));

    ROS_INFO("[biclops_controller]: PAN: %f, TILT: %f", panrev, tiltrev);

    panProfile.pos  = panrev;
    tiltProfile.pos = tiltrev;
}

int main (int argc, char *argv[]) {

    ros::init(argc,argv,"biclops_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber key_sub;
    ros::Subscriber joint_commands_sub;

    sensor_msgs::JointState joint_state;

    std::string config_path;
    nh.param<std::string>("biclops/config_file_path", config_path, "/src/biclops_controller/BiclopsDefault.cfg");

    ROS_INFO("[biclops_controller]: config_file_path param set to '%s'. Loading.", config_path.c_str());

    //Subscribing
    joint_commands_sub = nh.subscribe<biclops_controller::biclops_command>("/biclops/tip_orientation", 1, tip_orientation_callback);

    //Initializing
    if (!biclops.Initialize(config_path.c_str()))
    {
        ROS_ERROR("[biclops_controller]: failed to open biclops communication. Terminating.");
        return 0;
    }
    else
        ROS_INFO("[biclops_controller]: communication open. Listening on /biclops/joint_commands." );

    //tuck_biclops sequence at the beginning of the application
    biclops.HomeAxes(axisMask,true);

    // Get shortcut references to each axis.
    pan_axis  = biclops.GetAxis(Biclops::Pan);
    tilt_axis = biclops.GetAxis(Biclops::Tilt);

    // Get the currently defined (default) motion profiles.
    pan_axis->GetProfile(panProfile);
    tilt_axis->GetProfile(tiltProfile);


    while(ros::ok() ){

        pan_axis->SetProfile(panProfile);
        pan_axis->Move(false);
        tilt_axis->SetProfile(tiltProfile);
        tilt_axis->Move(false);

        spinOnce();
        loop_rate.sleep();

    }

    pan_axis->DisableAmp();
    tilt_axis->DisableAmp();

    return 0;

}

	    
