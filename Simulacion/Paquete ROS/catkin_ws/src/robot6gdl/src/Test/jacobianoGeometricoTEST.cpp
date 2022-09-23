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
    double tf = 10;
    double dt = 0.01;
    int numberElements = (tf-ti)/dt;
    Eigen::Array<double,1,-1> arregloTiempo{Eigen::Array<double,1,-1>::LinSpaced(numberElements,ti,tf)};

    std::cout<<"Generando las trayectoria articulares\n";
    //Generar arreglo para almacenar todas las qs
    Eigen::Array<double,6,Dynamic> Qs{6,arregloTiempo.size()};
    
    //Vector de condiciones iniciales
    Eigen::Matrix<double,4,6> cE{{0,0,0,0,0,0},
                                 {0,0,0,0,0,0},
                                {M_PI/2,M_PI/4,M_PI/2,-M_PI,M_PI/2,0},
                                {0,0,0,0,0,0}};

    //Vector para almacenar las trayectorias
    std::vector<Eigen::Array<double,6,Dynamic>> vectorQ;

    //Generar las trayectorias cada una con sus respectivas condiciones iniciales
     //Almacenar las q en Qs
    for(int i = 0; i < cE.cols(); i++)
    {
        Eigen::Vector4d E{cE.col(i)};
       Qs.row(i) = (generarTrayectoriaCubica(arregloTiempo,E));
    }    

    std::cout<<"Las trayectorias son:\n";
    std::cout<<Qs;



}