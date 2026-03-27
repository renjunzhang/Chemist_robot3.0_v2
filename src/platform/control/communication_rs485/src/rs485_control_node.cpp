#include <hf_utils.h>

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "rs485_control");

  ros::NodeHandle nh;
  ros::Rate loop_rate(ROSLOOPRATE);

  RS485Control rs485_controller(nh, portName, BAUDRATE);

  if (!rs485_controller.serialInit())
    return -1;

  while (ros::ok()) {
    rs485_controller.runOnce(loop_rate);
    ros::spinOnce();
  }

  return 0;
}
