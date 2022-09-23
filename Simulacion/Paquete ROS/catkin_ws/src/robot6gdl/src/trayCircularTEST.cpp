//Archivo para realizar la trayectoria circular
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "gTrayArt.h"

int main()
{
    std::cout<<"INICIANDO PROGRAMA\n";
    std::cout<<"Generando parámetros del circulo\n";
    //Centro del virculo
    Eigen::Vector3d c{200,0,200};

    //Posición inicial del circulo
    Eigen::Vector3d  pi{300,0,200};

    //Determinar el radio del circulo
    double rho = (pi-c).norm();

    //Matriz de rotación que proyecta el circulo en su eje local al eje global
    Eigen::Matrix3d R{{1,0,0},{0,0,-1},{0,1,0}};

    //Definir la longitud de arco final del circulo
    double sf = 2*M_PI*rho;

    //Generar arreglo de tiempo
    double ti = 0;
    double tf = 10;
    double dt = 0.01;
    int numberElements = (tf-ti)/dt;

    Eigen::Array<double,1,-1> arregloTiempo{Eigen::Array<double,1,-1>::LinSpaced(numberElements,ti,tf)};

    //Generar arreglo para almacenar el circulo
    Eigen::Array3Xd X_0H{3,arregloTiempo.size()};

    //Obtener la parametrización de S

    //Definir las condiciones iniciales
    Eigen::Vector4d cE{0.0,0.0,sf,0.0};

    Eigen::Array<double,1,-1> s{generarTrayectoriaCubica(arregloTiempo,cE)};

    X_0H.row(0) = rho*Eigen::sin(s/rho); 
    X_0H.row(1) = rho*Eigen::cos(s/rho);
    X_0H.row(2) = 0*Eigen::Array<double,1,-1>::Ones(arregloTiempo.size());

    std::cout<<"La trayectoria del circulo es :\n";
    std::cout<<X_0H;



}