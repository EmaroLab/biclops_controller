#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <Biclops.h>
#include <biclops_controller/biclops_joint_states.h>

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

biclops_controller::biclops_joint_states joint_state;
PMDAxisControl::Profile panProfile,tiltProfile;


void set_tip_orientation(double pan, double tilt){
    double rad_pan = -((pan + 45) * 2 * PI) / 360; // accounts for reference offset and clockwise rotation
    double rad_tilt = (tilt * 2 * PI) / 360;

    double panrev  = (rad_pan  / (2 * PI));
    double tiltrev = (rad_tilt / (2 * PI));

    ROS_INFO("[biclops_controller]: PAN: %f, TILT: %f", joint_state.pan, joint_state.tilt);

    panProfile.pos  = panrev;
    tiltProfile.pos = tiltrev;
}


void tip_orientation_callback(biclops_controller::biclops_joint_states msg){
    set_tip_orientation(msg.pan, msg.tilt);
}


int main (int argc, char *argv[]) {

    ros::init(argc,argv,"biclops_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber key_sub;
    ros::Subscriber joint_commands_sub;
    ros::Publisher joint_states_pub;

    std::string config_path;
    bool publish_frame_to_baxter_head;
    nh.param<std::string>("biclops/config_file_path", config_path, "/src/biclops_controller/BiclopsDefault.cfg");
    nh.param<bool>("biclops/baxter_tf", publish_frame_to_baxter_head, false);

    ROS_INFO("[biclops_controller]: config_file_path param set to '%s'. Loading.", config_path.c_str());

    //Initializing connections
    joint_commands_sub = nh.subscribe<biclops_controller::biclops_joint_states>("/biclops/tip_orientation", 1, tip_orientation_callback);
    joint_states_pub = nh.advertise<biclops_controller::biclops_joint_states>("/biclops/joint_states", 1);
    joint_state.pan  = 0;
    joint_state.tilt = 0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion frame_orientation;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.3) ); // TODO identify z shift, possibly also x and y

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

    PMDint32 tmp;

    set_tip_orientation(0.0, 0.0);

    while(ros::ok() ){

        pan_axis->SetProfile(panProfile);
        pan_axis->Move(false);
        tilt_axis->SetProfile(tiltProfile);
        tilt_axis->Move(false);

        pan_axis->GetPosition(tmp);
        joint_state.pan= -((double)tmp / 33 + 45); // accounts for reference offset and clockwise rotation
        tilt_axis->GetPosition(tmp);
        joint_state.tilt = (double)tmp / 33;
        joint_states_pub.publish(joint_state);

        if (publish_frame_to_baxter_head) {
            frame_orientation.setRPY(joint_state.tilt * PI / 180, 0.0, joint_state.pan * PI / 180);
            transform.setRotation(frame_orientation);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head", "biclops"));
        }

        spinOnce();
        loop_rate.sleep();
    }

    pan_axis->DisableAmp();
    tilt_axis->DisableAmp();

    return 0;

}

	    
