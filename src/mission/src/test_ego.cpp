#include <template.h>
#include <ego.h>


int mission_num = 0;   //mission_flag
float if_debug = 0;
float err_max = 0.2;
float ego_target_x_1 = 3.5;
float ego_target_y_1 = 2.2;
float ego_target_x_2 = -1.5;
float ego_target_y_2 = -2.2;
float ego_target_x_3 = 3.5;
float ego_target_y_3 = -2.2;
float ego_target_x_4 = -3.5;
float ego_target_y_4 = 2.2;

void print_param()
{
  cout << "=== 控制参数 ===" << endl;
  cout << "err_max: " << err_max << endl;
  cout << "ALTITUDE: " << ALTITUDE << endl;
  cout << "if_debug: " << if_debug << endl;
  if(if_debug == 1) cout << "自动offboard" << endl;
  else cout << "遥控器offboard" << endl;
  cout<<"ego_planner到达目标点误差阈值 ego_err_max: "<<ego_err_max<<endl;
  cout<<"================"<<endl;
  cout<<"ego_target_x_1: "<<ego_target_x_1<<endl;
  cout<<"ego_target_y_1: "<<ego_target_y_1<<endl;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "test_ego");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 订阅ego_planner规划出来的结果
  ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

  // 发布ego_planner目标
  planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

  ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

  finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

  // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // nh.param<float>("virtual_floor", virtual_floor,0.5);
  // nh.param<float>("virtual_ceiling", virtual_ceiling,2.0);
  nh.param<float>("ego_err_max", ego_err_max,0.2);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("ego_target_x_1", ego_target_x_1, 3.5);
  nh.param<float>("ego_target_y_1", ego_target_y_1, 2.2);

  print_param();
  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < err_max)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
 	      last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  



  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);
    
    switch (mission_num)
    {
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2))
        {
          if(lib_time_record_func(1.0, ros::Time::now()))
          {
            mission_num = 2;
            last_request = ros::Time::now();
          } 
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      case 2:
      if (pub_ego_goal(ego_target_x_1, ego_target_y_1, 0.8, ego_err_max))
      {
          last_request = ros::Time::now();
          mission_num = 6;
          ego_check = false;
      }
      break;

      case 3:
      if(pub_ego_goal(ego_target_x_2, ego_target_y_2, ALTITUDE, ego_err_max))
      {
          last_request = ros::Time::now();
          mission_num = 4;
          ego_check = false;
      }
      break;

      case 4:
      if(pub_ego_goal(ego_target_x_3, ego_target_y_3, ALTITUDE, ego_err_max))
      {
          last_request = ros::Time::now();
          mission_num = 5;
          ego_check = false;
      }
      break;

      case 5:
      if(pub_ego_goal(ego_target_x_4, ego_target_y_4, ALTITUDE, ego_err_max))
      {
          last_request = ros::Time::now();
          mission_num = 2;
          ego_check = false;
      }
      break;

      case 6:
      if(precision_land())
      {
          mission_num = -1;
      }

    }

    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    if(mission_num == -1) exit(0);
  }
  return 0;
}


