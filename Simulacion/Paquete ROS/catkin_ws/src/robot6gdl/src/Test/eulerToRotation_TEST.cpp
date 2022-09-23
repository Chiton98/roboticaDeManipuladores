#include <iostream>
#include "../moduloCinematica.h"


int main()
{

    //Definir los parámetros
    double alfa = 0;
    double beta = -M_PI/2;
    double gamma = -M_PI;

    //Mandar a llamar la funcion
    Eigen::Matrix3d R{eulerRPYToRotationMatrix(alfa,beta,gamma)};

    //Mostrar la matriz de rotación
    std::cout<<R;
}
