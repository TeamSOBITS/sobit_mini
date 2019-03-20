#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <sobit_mini_control/sobit_mini_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sobit_mini_control_node");
  SobitMiniControl sobit_mini_ctr;
  sobit_mini_ctr.openPort();
  sobit_mini_ctr.initializeDynamixel();
  sobit_mini_ctr.writeInitialJoint();
  controller_manager::ControllerManager cm(&sobit_mini_ctr, sobit_mini_ctr.nh_);
  //ros::Rate rate(1 / sobit_mini_ctr.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  //ros::MultiThreadedSpinner spinner(4);
  spinner.start();
  ros::Duration(4).sleep();

  while(ros::ok())
  {
    ros::Time now = sobit_mini_ctr.getTime();
    ros::Duration dt = sobit_mini_ctr.getPeriod();

    sobit_mini_ctr.read(now, dt);
    cm.update(now, dt);
    sobit_mini_ctr.write(now, dt);
  }
  spinner.stop();
  //spinner.spin();

  return 0;
}