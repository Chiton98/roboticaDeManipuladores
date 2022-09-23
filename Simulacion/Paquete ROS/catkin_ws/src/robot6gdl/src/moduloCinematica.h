#include <eigen3/Eigen/Dense>

#ifndef MODULO_CINEMATICA
#define MODULO_CINEMATICA

//Funcion para convertir de RPY A ROTATIONMATRIX
Eigen::Matrix3d eulerZYXToRotationMatrix(double alfa,double beta,double gamma);

//Funcion para convertir de RotationMatrix to RPY
Eigen::Vector3d rotationMatrixtoeulerZYX(Eigen::Matrix3d R);

//Funcion para obtener la matriz E, convierte razon de cambio de alfa,beta,gamm
//a velocidades angulares wx,wy,wz
Eigen::Matrix3d obtenerMatrizE(double alfa,double beta,double gamma);

#endif