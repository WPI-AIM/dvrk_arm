#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name){
    originFramePtr.reset(new Frame);
    eeFramePtr.reset(new Frame);
    afxdTipFramePtr.reset(new Frame);
    freeFramePtr.reset(new Frame);


    frameptrVec.push_back(originFramePtr);
    frameptrVec.push_back(eeFramePtr);
    frameptrVec.push_back(afxdTipFramePtr);

    m_bridge.reset(new DVRK_Bridge(arm_name));
    m_bridge->poseFcnHandle.assign_fcn(&DVRK_Arm::pose_fcn_cb, this);
    m_bridge->jointFcnHandle.assign_fcn(&DVRK_Arm::joint_state_fcn_cb, this);
    m_bridge->wrenchFcnHandle.assign_fcn(&DVRK_Arm::wrench_fcn_cb, this);
    m_bridge->gripperFcnHandle.assign_fcn(&DVRK_Arm::gripper_state_fcn_cb, this);
    counter = 0;
}

void DVRK_Arm::init(){
}

void DVRK_Arm::pose_fcn_cb(const geometry_msgs::PoseStamped &pose){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    freeFramePtr->pos.setX(pose.pose.position.x);
    freeFramePtr->pos.setY(pose.pose.position.y);
    freeFramePtr->pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, freeFramePtr->rot_quat);

    freeFramePtr->trans.setOrigin(freeFramePtr->pos);
    freeFramePtr->trans.setRotation(freeFramePtr->rot_quat);
    freeFramePtr->trans = originFramePtr->trans.inverse() * freeFramePtr->trans * afxdTipFramePtr->trans;
    eeFramePtr->trans = freeFramePtr->trans;
    handle_frames();
}

void DVRK_Arm::gripper_state_fcn_cb(const sensor_msgs::JointState &state){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    gripper_angle = state.position[0];
}

void DVRK_Arm::joint_state_fcn_cb(const sensor_msgs::JointState &jnt){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    if (jointPos.size() != jnt.position.size()){
        jointPos.resize(jnt.position.size());
    }
    if (jointVel.size() != jnt.velocity.size()){
        jointVel.resize(jnt.velocity.size());
    }
    if (jointEffort.size() != jnt.effort.size()){
        jointEffort.resize(jnt.effort.size());
    }
    jointPos = jnt.position;
    jointVel = jnt.velocity;
    jointEffort = jnt.effort;
}

void DVRK_Arm::wrench_fcn_cb(const geometry_msgs::WrenchStamped &wrench){

}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    tf_mat.getRotation(originFramePtr->rot_quat);
    set_origin_frame(pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    originFramePtr->trans.setOrigin(pos);
    originFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Transform &trans){
    originFramePtr->trans = trans;
}

void DVRK_Arm::set_origin_frame_pos(const double &x, const double &y, const double &z){
    originFramePtr->pos.setX(x);
    originFramePtr->pos.setY(y);
    originFramePtr->pos.setZ(z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const geometry_msgs::Point &pos){
    originFramePtr->pos.setX(pos.x);
    originFramePtr->pos.setY(pos.y);
    originFramePtr->pos.setZ(pos.z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const tf::Vector3 &pos){
    originFramePtr->pos = pos;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw){
    originFramePtr->rot_quat.setRPY(roll, pitch, yaw);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Quaternion &tf_quat){
    originFramePtr->rot_quat = tf_quat;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    tf::quaternionMsgToTF(gm_quat, originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Matrix3x3 &mat){
    mat.getRotation(originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const double &x, const double &y, const double &z){
    afxdTipFramePtr->pos.setX(x);
    afxdTipFramePtr->pos.setY(y);
    afxdTipFramePtr->pos.setZ(z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const geometry_msgs::Point &pos){
    afxdTipFramePtr->pos.setX(pos.x);
    afxdTipFramePtr->pos.setY(pos.y);
    afxdTipFramePtr->pos.setZ(pos.z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const tf::Vector3 &pos){
    afxdTipFramePtr->pos = pos;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw){
    afxdTipFramePtr->rot_quat.setRPY(roll, pitch, yaw);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w){
    afxdTipFramePtr->rot_quat.setX(quat_x);
    afxdTipFramePtr->rot_quat.setY(quat_y);
    afxdTipFramePtr->rot_quat.setZ(quat_z);
    afxdTipFramePtr->rot_quat.setW(quat_w);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    tf::quaternionMsgToTF(gm_quat, afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const tf::Quaternion &tf_quat){
    afxdTipFramePtr->rot_quat = tf_quat;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    afxdTipFramePtr->trans.setOrigin(pos);
    afxdTipFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    afxdTipFramePtr->pos = pos;
    tf_mat.getRotation(afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Transform &trans){
    afxdTipFramePtr->trans = trans;
}

void DVRK_Arm::measured_cp_pos(double &x, double &y, double &z){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    x = eeFramePtr->trans.getOrigin().getX();
    y = eeFramePtr->trans.getOrigin().getY();
    z = eeFramePtr->trans.getOrigin().getZ();
}

void DVRK_Arm::measured_cp_pos(tf::Vector3 &pos){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    pos = eeFramePtr->trans.getOrigin();
}

void DVRK_Arm::measured_cp_pos(geometry_msgs::Point &pos){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    tf::pointTFToMsg(eeFramePtr->trans.getOrigin(), pos);
}

void DVRK_Arm::measured_cp_ori(double &roll, double &pitch, double &yaw){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    tf::Matrix3x3(eeFramePtr->trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::measured_cp_ori(double &x, double &y, double &z, double &w){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    x = eeFramePtr->trans.getRotation().getX();
    y = eeFramePtr->trans.getRotation().getY();
    z = eeFramePtr->trans.getRotation().getZ();
    w = eeFramePtr->trans.getRotation().getW();
}

void DVRK_Arm::measured_cp_ori(tf::Quaternion &tf_quat){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    tf_quat = eeFramePtr->trans.getRotation();
    tf_quat.normalize();
}

void DVRK_Arm::measured_cp_ori(geometry_msgs::Quaternion &gm_quat){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), gm_quat);
}

void DVRK_Arm::measured_cp_ori(tf::Matrix3x3 &mat){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    mat.setRotation(eeFramePtr->trans.getRotation());
}

void DVRK_Arm::measured_cp(geometry_msgs::Pose &pose){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    pose.position.x = eeFramePtr->trans.getOrigin().getX();
    pose.position.y = eeFramePtr->trans.getOrigin().getY();
    pose.position.z = eeFramePtr->trans.getOrigin().getZ();

    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), pose.orientation);
}

void DVRK_Arm::measured_cp(tf::Transform &trans){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    trans = eeFramePtr->trans;
    trans.setRotation(trans.getRotation().normalized());
}

void DVRK_Arm::measured_jp(std::vector<double> &jnt_pos) {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    if (jnt_pos.size() != jointPos.size()){
        jnt_pos.resize(jointPos.size());
    }
    jnt_pos = jointPos;
}

void DVRK_Arm::measured_jv(std::vector<double> &jnt_vel) {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    if (jnt_vel.size() != jointVel.size()){
        jnt_vel.resize(jointVel.size());
    }
    jnt_vel = jointPos;
}

void DVRK_Arm::measured_jf(std::vector<double> &jnt_effort) {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    if (jnt_effort.size() != jointEffort.size()){
        jnt_effort.resize(jointEffort.size());
    }
    jnt_effort = jointPos;
}

void DVRK_Arm::measured_gripper_angle(double &pos){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    pos = gripper_angle;
}

bool DVRK_Arm::move_cp_pos(const double &x, const double &y, const double &z){
    eeCmd.trans.setOrigin(tf::Vector3(x,y,z));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_pos(const geometry_msgs::Point &pos){
    eeCmd.trans.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_pos(const tf::Vector3 &pos){
    eeCmd.trans.setOrigin(pos);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_ori(const double &roll, const double &pitch, const double &yaw){
    eeCmd.rot_quat.setRPY(roll, pitch, yaw);
    eeCmd.trans.setRotation(eeCmd.rot_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_ori(const double &x, const double &y, const double &z, const double &w){
    eeCmd.trans.setRotation(tf::Quaternion(x,y,z,w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_ori(const tf::Quaternion &tf_quat){
    eeCmd.trans.setRotation(tf_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_ori(const geometry_msgs::Quaternion &gm_quat){
    eeCmd.trans.setRotation(tf::Quaternion(gm_quat.x, gm_quat.y, gm_quat.z, gm_quat.w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp_ori(const tf::Matrix3x3 &mat){
    mat.getRotation(eeCmd.rot_quat);
    eeCmd.trans.setRotation(eeCmd.rot_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp(geometry_msgs::PoseStamped &pose){
    eeCmd.trans.setOrigin(tf::Vector3(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z));

    eeCmd.trans.setRotation(tf::Quaternion(pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z,
                             pose.pose.orientation.w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::move_cp(tf::Transform &trans){
    eeCmd.trans = trans;
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::is_gripper_pressed(){
    return m_bridge->_gripper_closed;
}

bool DVRK_Arm::is_clutch_pressed(){
    return m_bridge->_clutch_pressed;
}

bool DVRK_Arm::is_coag_pressed(){
    return m_bridge->_coag_pressed;
}

void DVRK_Arm::set_mode(const std::string &state, bool lock_wrench_ori){
    m_bridge->set_cur_mode(state, lock_wrench_ori);
}

void DVRK_Arm::move_arm_cartesian(tf::Transform trans){
    geometry_msgs::PoseStamped cmd_pose;
    trans = originFramePtr->trans * trans * afxdTipFramePtr->trans.inverse();
    cmd_pose.pose.position.x = trans.getOrigin().getX();
    cmd_pose.pose.position.y = trans.getOrigin().getY();
    cmd_pose.pose.position.z = trans.getOrigin().getZ();
    tf::quaternionTFToMsg(trans.getRotation().normalized(), cmd_pose.pose.orientation);

    m_bridge->set_cur_pose(cmd_pose);
}

bool DVRK_Arm::set_force(const double &fx, const double &fy, const double &fz){
    eeCmd.force.setX(fx);
    eeCmd.force.setY(fy);
    eeCmd.force.setZ(fz);

    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

bool DVRK_Arm::set_moment(const double &nx, const double &ny, const double &nz){
    eeCmd.moment.setX(nx);
    eeCmd.moment.setY(ny);
    eeCmd.moment.setZ(nz);

    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

bool DVRK_Arm::set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz){
    eeCmd.force.setX(fx);
    eeCmd.force.setY(fy);
    eeCmd.force.setZ(fz);
    eeCmd.moment.setX(nx);
    eeCmd.moment.setY(ny);
    eeCmd.moment.setZ(nz);
    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

void DVRK_Arm::set_arm_wrench(tf::Vector3 &force, tf::Vector3 &moment){
    if(m_bridge->_clutch_pressed == true){
        force.setZero();
        moment.setZero();
    }
    originFramePtr->rot_mat.setRotation(originFramePtr->trans.getRotation());
    geometry_msgs::Wrench cmd_wrench;
    tf::vector3TFToMsg(originFramePtr->rot_mat * force, cmd_wrench.force);
    tf::vector3TFToMsg(originFramePtr->rot_mat * moment, cmd_wrench.torque);
    m_bridge->set_cur_wrench(cmd_wrench);
}

void DVRK_Arm::handle_frames(){
    for(frameIter = frameptrVec.begin(); frameIter !=frameptrVec.end(); frameIter++){
        double x,y,z;
        x = (*frameIter)->trans.getOrigin().getX();
        y = (*frameIter)->trans.getOrigin().getY();
        z = (*frameIter)->trans.getOrigin().getZ();

        double qx, qy, qz, qw;
        qx = (*frameIter)->trans.getRotation().getX();
        qy = (*frameIter)->trans.getRotation().getY();
        qz = (*frameIter)->trans.getRotation().getZ();
        qw = (*frameIter)->trans.getRotation().getW();
        if (std::isnan(x) || std::isnan(y) ||  std::isnan(z)){
            (*frameIter)->trans.setOrigin(tf::Vector3(0,0,0));
            std::cerr<< "Origin of frame is NAN, setting origin to (0,0,0)" << std::endl;
        }
        if (std::isnan(qx) || std::isnan(qy) ||  std::isnan(qz) || std::isnan(qw)){
            (*frameIter)->trans.setRotation(tf::Quaternion().getIdentity());
            std::cerr<< "Rotation of frame is NAN, setting rotation to (0,0,0)" << std::endl;
        }
        //Normalize the rotation quaternion;
        (*frameIter)->trans.getRotation() = (*frameIter)->trans.getRotation().normalized();
        //Setting the pos, quat and rot_mat members of the struct so all of them have the same data as trans
        (*frameIter)->pos = (*frameIter)->trans.getOrigin();
        (*frameIter)->rot_quat = (*frameIter)->trans.getRotation();
        (*frameIter)->rot_mat.setRotation((*frameIter)->rot_quat);
    }
    counter++;
    if (counter % 15 == 0){
//    frame_broadcaster.sendTransform(tf::StampedTransform(originFramePtr->trans, ros::Time::now(), "world", "arm_origin"));
//    frame_broadcaster.sendTransform(tf::StampedTransform(eeFramePtr->trans, ros::Time::now(), "arm_origin", "ee"));
    counter = 0;
    }
}

bool DVRK_Arm::close(){
    m_bridge->shutDown();
    return true;
}

DVRK_Arm::~DVRK_Arm(){
    std::cerr << "CLOSING DVRK_ARM" << std::endl;
}


extern "C"{
std::vector<std::string> get_active_arms(){
    std::vector<std::string> active_arm_names;
    DVRK_Bridge::get_arms_from_rostopics(active_arm_names);
    return active_arm_names;
}
boost::shared_ptr<DVRK_Arm> create(std::string arm_name){
    return boost::shared_ptr<DVRK_Arm>(new DVRK_Arm(arm_name));
}
void destroy(boost::shared_ptr<DVRK_Arm> arm_obj){
    arm_obj.reset();
}
}
