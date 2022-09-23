#include <iostream>
#include <eigen3/Eigen/Dense>
#include "claseRobots.h"
#include <string>
using namespace Eigen;
using namespace std;

int main(int argc,char *argv[])
{
    
    //Obtener los valores de la linea de comandos
    double x0 = std::stod(argv[1]);
    double z0 = std::stod(argv[2]);
    double r = std::stod(argv[3]);

    string imprimir = argv[4];
    std::cout<<"Generando los parametros del vector tiempo\n";

    //Generar parámetros para el vector tiempo
    double ti = 0;
    double tf = 5.0;
    double dt = 0.01;

    std::cout<<"Generando los parametros de la trayectoria\n";
    //Generar los parámetros para la trayectoria
    double y0 = 20.0;
    double w = 2*M_PI/tf;

    int number_elements = (tf-ti)/dt;

    std::cout<<"Generando el X_0H\n";
    //Generar tabla
    ArrayXXd X_0H(3,number_elements+1); //Allocating memory for the fixed array
    std::cout<<"Generando arreglo de tiempos\n";
    //Generar arreglo de tiempos
    Array<double,1,Dynamic> arrayTime = Array<double,1,Dynamic>::LinSpaced(number_elements+1,ti,tf);

    std::cout<<"Generando trayectoria en espacio cartesiano\n";
    //Generar arreglo para la posicion en X
    X_0H.row(0) = r*cos(w*arrayTime)+x0;

    //Generar arreglo para la posicion en Y
    X_0H.row(1) = y0*Array<double,1,Dynamic>::Ones(number_elements+1);
    
    //Generar arreglo para la posicion en Z
    X_0H.row(2) = r*Eigen::sin(w*arrayTime)+z0;

    //Generar arreglo que contendrá los ángulos

    ArrayXXd Qs(3,number_elements+1);

    //Crear el robot
    Robot6DOF instanciaRobot{};

    std::cout<<"Generando trayectoria en espacio articular\n";
    //Iterar para evaluar los valores de la herramienta
    for(int i = 0; i < number_elements+1; i++)
    {
        Qs.col(i) = instanciaRobot.obtenerAngulosArticulares(Matrix4d{ {-1.0,0.0,0.0,X_0H(0,i)},
                                                                        {0.0,1.0,0.0,X_0H(1,i)},
                                                                        {0.0,0.0,1.0,X_0H(2,i)},
                                                                        {0.0,0.0,0.0,1.0}});
    }

 

    //Iterar para obtener los angulos articulares
   
    return 0;

}