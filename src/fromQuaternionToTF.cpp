#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "from_quaternion_to_tf");


  tf::Quaternion quat(-0.9902134044596855, -0.005515230213965462, 0.019933823420209988, 0.13802042800940023);

   // the tf::Quaternion has a method to acess roll pitch and yaw
   double roll, pitch, yaw;
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  std::cout << "Roll: " << roll << " - Pitch: " << pitch << " - Yaw: " << yaw << std::endl;


  return 0;
}
