//Archivo para realizar la trayectoria circular
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "gTrayArt.h"

int main()
{
    std::cout<<"INICIANDO PROGRAMA\n";
    //Generar arreglo de tiempo
    std::cout<<"Generando el arreglo de tiempo\n";
    double ti = 0;
    double tf = 2;
    double dt = 0.01;
    int numberElements = (tf-ti)/dt;
    Eigen::Array<double,1,-1> arregloTiempo{Eigen::Array<double,1,-1>::LinSpaced(numberElements,ti,tf)};

    std::cout<<"Generando las trayectoria articulares\n";
    //Generar arreglo para almacenar todas las posiciones angulares
    Eigen::Array<double,6,Dynamic> Qs{6,arregloTiempo.size()};
    //Generar arreglo para almacenar todas las velocidades angulares
    Eigen::Array<double,6,Dynamic> Qds{6,arregloTiempo.size()};
    
    //Vector de condiciones iniciales
    Eigen::Matrix<double,4,6> cE{{M_PI/20,M_PI/20,M_PI/20,M_PI/20,M_PI/20,M_PI/20},
                                 {0,0,0,0,0,0},
                                {M_PI/2,M_PI/4,M_PI/2,-M_PI,M_PI/2,0},
                                {0,0,0,0,0,0}};

 

    //Generar las trayectorias cada una con sus respectivas condiciones iniciales
     //Almacenar las q en Qs
    for(int i = 0; i < cE.cols(); i++)
    {
        Eigen::Vector4d E{cE.col(i)};
       Qs.row(i) = (generarTrayectoriaCubicaModificada(arregloTiempo,E))[0];
       Qds.row(i) = (generarTrayectoriaCubicaModificada(arregloTiempo,E))[1];
    }

    //Generar arreglo para almacenar los vectores del Twist
    Eigen::Array<double,6,Dynamic> X_0H{6,arregloTiempo.size()};

    //Generar robot de 6 DOF
    std::cout<<"Crear robot 6DOF\n";
    Robot6DOF myRobot{};
    
    for(int i = 0; i < Qs.cols();i++)
    {

            X_0H.col(i) = myRobot.obtenerJacobianoGeometrico(Qs.col(i))*(Qds.col(i).matrix()); 

    }
    std::cout<<"Se multiplica la velocidad angular por el jacobiano:\n";


    std::cout<<"Las trayectorias son:\n";
    for(int i = 0;i < Qs.rows();i++)
    {   std::cout<<"Se imprime el renglón "<<i<<"\n";
        std::cout<<X_0H.row(i);
    }
    
   



}