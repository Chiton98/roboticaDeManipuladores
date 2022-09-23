#include <iostream>
#include <eigen3/Eigen/Core>
#include "gTrayArt.h"

using namespace Eigen;

int main()
{
  //1.-Generar la trayectoria
  //Definir posicion inicial y final
	Vector3d pi{100.0,0.0,53.0};
	Vector3d pf{250.0,0.0,100.0};

  //Generar la trayectoria de linea recta
  Eigen::Array<double,3,Dynamic> X_0H = generarTrayectoriaLineaRecta(pi,pf);


  //Imprimir los valores
  std::cout<<X_0H;

  return 0;
}

