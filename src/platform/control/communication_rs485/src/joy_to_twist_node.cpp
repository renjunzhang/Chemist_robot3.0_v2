#include <hf_utils.h>

using namespace std;

class Teleop {

public:
  Teleop();

private:
  /* data */

  void callback(const sensor_msgs::Joy::ConstPtr &Joy);

  ros::NodeHandle nh; //实例化节点
  ros::Subscriber subJoy;
  ros::Publisher pubVel;
  ros::Publisher pubPauseNavFlag;

  double linear_x, linear_y, angular; //我们控制乌龟的速度，是通过这两个变量调整

  int axis_ang, axis_linear_x, axis_linear_y; // axes[]的键

  int charge_button;
  int clear_button;

  int enable_button;
  int release_nav_button;
};

Teleop::Teleop() {
  pubVel = nh.advertise<geometry_msgs::Twist>("joy_vel", 2);
  pubPauseNavFlag = nh.advertise<std_msgs::Bool>("pause_navigation", 1);
  subJoy = nh.subscribe<sensor_msgs::Joy>("joy", 2, &Teleop::callback, this);

  //我们将这几个变量加上参数，可以在参数服务器方便修改

  nh.param<int>("joy_to_twist/axis_linear_x", axis_linear_x,
                0); //默认axes[0]接收x速度
  nh.param<int>("joy_to_twist/axis_linear_y", axis_linear_y,
                1); //默认axes[1]接收y速度
  nh.param<int>("joy_to_twist/axis_angular", axis_ang, 2); //默认axes[2]接收角度

  nh.param<double>("joy_to_twist/scale_linear", linear_x, 1); //默认线速度1 m/s
  nh.param<double>("joy_to_twist/scale_linear", linear_y, 1);
  nh.param<double>("joy_to_twist/scale_angular", angular,
                   1); //默认角速度1 单位rad/s

  nh.param<int>("joy_to_twist/charge_button", charge_button,
                1); //手动测试自动充电： 按下手柄 A 键激活充电
  nh.param<int>("joy_to_twist/clear_button", clear_button,
                0); //手动清除里程计

  nh.param<int>("joy_to_twist/enable_button", enable_button,
                5); // 按下手柄 RB 键激活控制
  nh.param<int>("joy_to_twist/release_nav_button", release_nav_button,
                4);  // 释放手柄控制，使能导航速度
}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr &Joy) {

  geometry_msgs::Twist v;
  std_msgs::Bool pause_nav_flag;

  if (Joy->buttons[enable_button]) {
    v.linear.x = Joy->axes[axis_linear_y] * linear_y; // 根据手柄调整正负号
    v.linear.y = Joy->axes[axis_linear_x] * linear_x;
    v.angular.z = Joy->axes[axis_ang] * angular;

    if (Joy->buttons[charge_button]) {
      v.angular.x = 1; // 角速度 x 分量设置为充电标志位
    } 
    if (Joy->buttons[clear_button]) {
      v.angular.y = 1; // 角速度 y 分量设置为清除里程计
    }
    pubVel.publish(v);

    // 双键暂停导航功能，暂时不用
    pause_nav_flag.data = true;
    pubPauseNavFlag.publish(pause_nav_flag);
  }

  // 双键暂停导航功能，暂时不用
  if (Joy->buttons[release_nav_button]) {
    pause_nav_flag.data = false;
    pubPauseNavFlag.publish(pause_nav_flag);

    pubVel.publish(v);
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "joy_to_twist");

  Teleop teleop_turtle;

  ros::spin();

  return 0;
}