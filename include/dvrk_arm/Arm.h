#ifndef CDVRK_ArmH
#define CDVRK_ArmH

#include "Bridge.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "boost/shared_ptr.hpp"
#include "dvrk_arm/Frame.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

struct Command: public Frame{
public:
    Command(){
        force.setZero();
        moment.setZero();
    }
    ~Command(){}
    tf::Vector3 force;
    tf::Vector3 moment;
};


class DVRK_Arm: public States{
public:
    DVRK_Arm(const std::string &arm_name);
    ~DVRK_Arm();

    void set_origin_frame_pos(const double &x, const double &y, const double &);
    void set_origin_frame_pos(const geometry_msgs::Point &pos);
    void set_origin_frame_pos(const tf::Vector3 &pos);

    void set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw);
    void set_origin_frame_rot(const tf::Quaternion &tf_quat);
    void set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void set_origin_frame_rot(const tf::Matrix3x3 &mat);

    void set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void set_origin_frame(const tf::Transform &trans);

    void affix_tip_frame_pos(const double &x, const double &y, const double &z);
    void affix_tip_frame_pos(const tf::Vector3 &pos);
    void affix_tip_frame_pos(const geometry_msgs::Point &pos);

    void affix_tip_frame_rot(const tf::Quaternion &tf_quat);
    void affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w);
    void affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw);

    void affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void affix_tip_frame(const tf::Transform &trans);

    bool set_force(const double &fx,const double &fy,const double &fz);
    bool set_moment(const double &nx,const double &ny,const double &nz);
    bool set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz);

    bool move_cp_pos(const double &x, const double &y, const double &z);
    bool move_cp_pos(const geometry_msgs::Point &pos);
    bool move_cp_pos(const tf::Vector3 &pos);

    bool move_cp_ori(const double &roll, const double &pitch, const double &yaw);
    bool move_cp_ori(const double &x, const double &y, const double &z, const double &w);
    bool move_cp_ori(const tf::Quaternion &tf_quat);
    bool move_cp_ori(const geometry_msgs::Quaternion &gm_quat);
    bool move_cp_ori(const tf::Matrix3x3 &mat);

    bool move_cp(geometry_msgs::PoseStamped &pose);
    bool move_cp(tf::Transform &trans);

    void measured_cp_pos(double &x, double &y, double &z);
    void measured_cp_pos(tf::Vector3 &pos);
    void measured_cp_pos(geometry_msgs::Point &pos);

    void measured_cp_ori(double &roll, double &pitch, double &yaw);
    void measured_cp_ori(double &x, double &y, double &z, double &w);
    void measured_cp_ori(tf::Quaternion &tf_quat);
    void measured_cp_ori(geometry_msgs::Quaternion &gm_quat);
    void measured_cp_ori(tf::Matrix3x3 &mat);

    void measured_cp(geometry_msgs::Pose &pose);
    void measured_cp(tf::Transform &trans);

    void measured_gripper_angle(double &pos);

    bool is_gripper_pressed(); //Presed or Released, for MTM
    bool is_clutch_pressed();
    bool is_coag_pressed();

    void set_mode(const std::string &state, bool lock_wrench_ori = true);

    bool _is_available(){return true;}
    bool _in_effort_mode(){return true;}
    bool _in_cart_pos_mode(){return true;}
    bool _in_jnt_pos_mode(){return m_bridge->_in_jnt_pos_mode();}

    bool start_pubs;
    bool gripper_closed;

    bool close();

private:

    void init();
    void handle_frames();
    void cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose);
    void cisstGripper_to_userGripper(const std_msgs::Float32 &pos);
    void cisstJoint_to_userJoint(const sensor_msgs::JointState &jnt);
    void cisstWrench_to_userWrench(const geometry_msgs::WrenchStamped &wrench);
    void userPose_to_cisstPose(geometry_msgs::PoseStamped &pose);
    void move_arm_cartesian(tf::Transform trans);
    void set_arm_wrench(tf::Vector3 &force, tf::Vector3 &wrench);
    // afxdTipFrame is the affixedTipFrame;

    typedef boost::shared_ptr<Frame> FramePtr;
    FramePtr originFramePtr, afxdTipFramePtr, eeFramePtr, freeFramePtr;
    Command eeCmd;
    std::vector<FramePtr> frameptrVec;
    std::vector<FramePtr>::iterator frameIter;
//    boost::shared_ptr<tf::TransformBroadcaster> frame_broadcasterPtr;
    double gripper_angle;
    int counter;

    boost::shared_ptr<DVRK_Bridge> m_bridge;

};
#endif
