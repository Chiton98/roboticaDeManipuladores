#include <eigen3/Eigen/Core>
#include <iostream>

using namespace Eigen;
using namespace std;

int main()
{

	
	//Definir el arreglo de 3x1
	Vector3d pi{100.0,0.0,53.0};
	Vector3d pf{250.0,0.0,100.0};
	
	Vector3d diff = pf-pi;
	
	//Arreglo de tiempos
	Array<double,1,Dynamic> arrayTime = Array<double,1,Dynamic>::LinSpaced(100,0,0.01);
	
	//Obtener la longitud de arco final
	double sf = (pf-pi).norm();
	
	//Obtener s(t)
	Array<double,1,Dynamic> sT = 2*arrayTime;
	
	//Posicion en X
	Array<double,1,Dynamic> positionX = pi(0) + diff(0)/sf*sT;
	
	//Posicion en Y
	Array<double,1,Dynamic> positionY = pi(1) + diff(1)/sf*sT;
	
	//Posicion en Z
	Array<double,1,Dynamic> positionZ = pi(2) + diff(2)/sf*sT;
	
	//Guardar todas las posiciones en un solo arreglo
	Array<double,3,Dynamic> X_0H(3,arrayTime.size());
	
	//Generar las columnas
	X_0H.row(0) = positionX;
	X_0H.row(1) = positionY;
	X_0H.row(2) = positionZ;
		

	cout<<X_0H;
	
	
	return 0;
}
