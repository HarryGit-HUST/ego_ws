#include "quadrotor_msgs/PositionCommand.h"

/************************************************************************
参数
*************************************************************************/
float ego_err_max = 0.2; // ego_planner到达目标点误差阈值

ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
std_msgs::Bool finish_ego_flag;

/************************************************************************
函数1 :ego_planner导航
*************************************************************************/
quadrotor_msgs::PositionCommand ego_sub;
uint32_t last_seq = -1;
double ego_now_x = 0;
double ego_now_y = 0;
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
	ego_sub = *msg;
}

/************************************************************************
函数2 : ego_planner是否规划出航线
*************************************************************************/
bool rec_traj_flag = false;
float last_ego_sub_x = 0;
float last_ego_sub_y = 0;
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg);
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg)
{
	rec_traj_flag = msg->data;
	// ROS_WARN("rec_traj_flag: %d", rec_traj_flag);
}
/************************************************************************
函数3 : 调整ego传入的目标点，PI控制器
*************************************************************************/
void PI_attitude_control()
{
	if(ego_sub.header.seq == last_seq){
		ROS_WARN("ego_sub.header.seq == last_seq, 未收到新的目标点");
		return;
	}                                                     
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.type_mask = 0b101111100011; // vx vy z yaw
	setpoint_raw.velocity.x = 0 * ego_sub.velocity.x + (ego_sub.position.x - local_pos.pose.pose.position.x) * 1.6;
	setpoint_raw.velocity.y = 0 * ego_sub.velocity.y + (ego_sub.position.y - local_pos.pose.pose.position.y) * 1.6;
	setpoint_raw.position.z = ego_sub.position.z;
	setpoint_raw.yaw = ego_sub.yaw;

	ROS_INFO("ego: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", ego_sub.velocity.x, ego_sub.velocity.y, ego_sub.position.z, ego_sub.yaw);
	ROS_INFO("ego_sub.position: x = %.2f, y = %.2f", ego_sub.position.x, ego_sub.position.y);
	ROS_INFO("已触发控制器: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.position.z, setpoint_raw.yaw);
	ROS_INFO("ego_target_x = %.2f, ego_target_y = %.2f", ego_now_x, ego_now_y);
	ROS_INFO("seq=%d",ego_sub.header.seq);
	last_ego_sub_x = ego_sub.position.x;
	last_ego_sub_y = ego_sub.position.y;
	last_seq = ego_sub.header.seq;
}

/************************************************************************
函数4 :ego_planner发布目标点函数
*************************************************************************/
float before_ego_pose_x = 0;
float before_ego_pose_y = 0;
float before_ego_pose_z = 0;
bool ego_check = false;
ros::Time last_sub_request;
bool pub_ego_goal_flag = false;
bool pub_ego_goal(float x, float y, float z, float err_max);
bool pub_ego_goal(float x, float y, float z, float err_max)
{
	if(ego_check == false){
		if(current_position_cruise(0, 0, ALTITUDE, 0, err_max)){
			current_position_cruise_flag = false;
			ego_check = true;
		}
		// cout << "ego_checking" << endl;
		ROS_WARN("ego_checking");
		return false;
	}
	before_ego_pose_x = local_pos.pose.pose.position.x;
	before_ego_pose_y = local_pos.pose.pose.position.y;
	before_ego_pose_z = local_pos.pose.pose.position.z;

	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	geometry_msgs::PoseStamped target_point;
	if (pub_ego_goal_flag == false)
	{
		pub_ego_goal_flag = true;;
		target_point.pose.position.x = x;
		target_point.pose.position.y = y;
		target_point.pose.position.z = z;
		ego_now_x = x;
		ego_now_y = y;

		planner_goal_pub.publish(target_point);
		ROS_INFO("发送目标点( %f, %f, %f )", x, y, z);

		finish_ego_flag.data = false;
		finish_ego_pub.publish(finish_ego_flag);
		last_sub_request = ros::Time::now();
	}

	if (rec_traj_flag == true)
	{
		PI_attitude_control();
	}
	else
    {
		std::cout << "not rec_traj" << std::endl;
		setpoint_raw.position.x = before_ego_pose_x;
		setpoint_raw.position.y = before_ego_pose_y;
		setpoint_raw.position.z = before_ego_pose_z;
		setpoint_raw.yaw = current_yaw;
		setpoint_raw.type_mask = 0b101111111000; // 101 111 111 000  x y z+ yaw
		setpoint_raw.coordinate_frame = 1;
	}

	setpoint_raw.header.stamp = ros::Time::now();

	if (fabs(local_pos.pose.pose.position.x - x) < err_max && fabs(local_pos.pose.pose.position.y - y) < err_max)
	{
		ROS_INFO("到达目标点, ego_planner导航任务完成");
		pub_ego_goal_flag = false;
		last_ego_sub_x = ego_sub.position.x;
		last_ego_sub_y = ego_sub.position.y;
		finish_ego_flag.data = true;
		finish_ego_pub.publish(finish_ego_flag);
		return true;
	}
	return false;
}