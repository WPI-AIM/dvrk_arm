#ifndef CDVRK_BRIDGEH
#define CDVRK_BRIDGEH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/WrenchStamped.h"
#include "FootPedals.h"
#include "Console.h"
#include "string.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "ros/callback_queue.h"
#include "dvrk_arm/States.h"
#include "FcnHandle.h"
#include "dvrk_arm/Timing.h"
#include "boost/thread.hpp"

class DVRK_Bridge: public States, public DVRK_FootPedals{
public:
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name, int bridge_frequnce = 1000);
    ~DVRK_Bridge();

    void set_cur_pose(const geometry_msgs::PoseStamped &pose);
    void set_cur_wrench(const geometry_msgs::Wrench &wrench);
    void set_cur_joint(const sensor_msgs::JointState &jnt_state);
    void set_cur_mode(const std::string &state, bool lock_ori);

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    static void get_arms_from_rostopics(std::vector<std::string> &arm_names);

    bool _start_pubs;
    bool _gripper_closed;

    typedef boost::shared_ptr<ros::NodeHandle> NodePtr;
    typedef boost::shared_ptr<ros::Rate> RatePtr;
    typedef boost::shared_ptr<ros::AsyncSpinner> AspinPtr;

    bool shutDown();

    FcnHandle<const geometry_msgs::PoseStamped&> poseFcnHandle;
    FcnHandle<const sensor_msgs::JointState&> jointFcnHandle;
    FcnHandle<const geometry_msgs::WrenchStamped&> wrenchFcnHandle;
    FcnHandle<const sensor_msgs::JointState&> gripperFcnHandle;

private:
    std::string arm_name;

    NodePtr n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_lock_pub;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
    ros::Publisher joint_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Subscriber wrench_sub;
    ros::Subscriber gripper_sub;
    ros::Subscriber gripper_angle_sub;
    ros::CallbackQueue cb_queue;
    RatePtr run_loop_rate, wrench_loop_max_rate;
    int _freq;

    double scale;
    std::vector<std::string> valid_arms;
    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void wrench_sub_cb(const geometry_msgs::WrenchStampedConstPtr &wrench);
    void gripper_sub_cb(const std_msgs::BoolConstPtr &gripper);
    void gripper_state_sub_cb(const sensor_msgs::JointStateConstPtr &state);
    void timer_cb(const ros::TimerEvent&);
    void _rate_sleep();
    void run();
    boost::shared_ptr<boost::thread> loop_thread;

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint, cmd_joint;
    std_msgs::String cur_state, state_cmd;
    geometry_msgs::WrenchStamped cur_wrench, cmd_wrench;
    bool _on;
};

#endif
