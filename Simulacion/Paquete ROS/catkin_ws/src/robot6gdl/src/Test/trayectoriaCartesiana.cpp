#include <iostream>
#include <eigen3/Eigen/Core>
using namespace Eigen;
using namespace std;

int main()
{

    //Generar parámetros para el vector tiempo
    double ti = 0;
    double tf = 5.0;
    double dt = 0.01;

    //Generar los parámetros para la trayectoria
    double x0 = 200;
    double z0 = 200;
    double y0 = 20.0;
    double r = 100.0;
    double w = 2*M_PI/tf;

    int number_elements = (tf-ti)/dt;

    //Generar tabla
    ArrayXXf table(number_elements+1,4); //Allocating memory for the fixed array

    //Generar arreglo de tiempos
    ArrayXf arrayTime = ArrayXf::LinSpaced(number_elements+1,ti,tf);
    table.col(0) = arrayTime;

    //Generar arreglo para la posicion en X
    table.col(1) = x0 + r*Eigen::cos(w*arrayTime);

    //Generar arreglo para la posicion en Y
    table.col(2) =  y0*ArrayXf::Ones(number_elements+1);
    
    //Generar arreglo para la posicion en Z
    table.col(3) = z0 + r*Eigen::sin(w*arrayTime);
    
    std::cout<<"Tiempo(s) \t X(m) \t Y(m) \t Z(m)\n";
    std::cout<<table<<std::endl;   
   
    return 0;

}