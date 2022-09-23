#ifndef G_TRAY_ART_H
#define G_TRAY_ART_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "claseRobots.h"
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

//
void publicarMarker(ros::Publisher &marker_publisher,Array3Xd &X_0H);

//Función que genera un circulo
std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaCircular(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo);

//Función que genera una cicloide
std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaPrimitiva(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo,int &opcion);


//Función para generar una linea recta
std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaLineaRecta(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo);

//Función para generar una linea recta con orientacion incluida
std::vector<Eigen::Array<double,6,Dynamic>> generarTrayectoriaLineaRectaOrientacion(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo);

//Función para interpolar los polinomios
Array<double,1,Dynamic> generarTrayectoriaCubica(Eigen::Array<double,1,Dynamic>& arregloTiempo,Eigen::Vector4d &cE);

//Función para interpolar los polinomios
std::vector<Array<double,1,Dynamic>> generarTrayectoriaCubicaModificada(Eigen::Array<double,1,Dynamic>& arregloTiempo,Eigen::Vector4d &cE);

#endif
