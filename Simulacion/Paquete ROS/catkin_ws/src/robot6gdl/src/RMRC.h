#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include <string>
#include "gTrayArt.h"
#include <eigen3/Eigen/Dense>

//Funcion principal para el RMRC
Eigen::Matrix<double,6,Dynamic> mainRMRC();

