#include "claseRobots.h"
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;


//Redefinir la funcion obtenerAngulosARticulares
Array<double,3,1> Robot6DOF::obtenerAngulosArticulares(Vector3d P_0H)
{

    //Extraer la posicion del centro de la herramienta
    double Xc = P_0H(0);
    double Yc = P_0H(1);
    double Zc = P_0H(2);
    //Extraer R_0H

    // <---------------------ÁNGULOS PARA EL CUERPO DEL ROBOT ----------------> 

    //3.-Utilizar la cinematica inversa para obtener los angulos
    //Angulo q1
    double q1 = atan2(Yc,Xc);
        
    //Angulo para la Articulacion 3
    double r = hypot(Xc,Yc);
    double s = Zc-_d1; 

    //Variable auxiliar D
    double D = (r*r+s*s-(_a2*_a2)-(_a3*_a3))/(2*_a2*_a3);

    //Obtener q3 por atan2
    double q3 = atan2(-pow(1-D*D,0.5),D); //Intentamos con la raiz positiva de s3
        
    //Tomar la solucion positiva de la articulacion 2
    double q2 = atan2(s,r) - atan2(_a3*sin(q3),_a2+_a3*cos(q3));

    return Array<double,3,1>{q1,q2,q3};
    }
    
//Función de la cinemática directa 
Matrix<double,4,4> Robot6DOF::obtenerCinematicaDirecta(Matrix<double,6,1> &Q)
{

	//Obtener los elementos de Q
	double q1 = Q(0);
	double q2 = Q(1);
	double q3 = Q(2);
	double q4 = Q(3);
	double q5 = Q(4);
	double q6 = Q(5);
	
	//Matriz de rotacion
	
	//Primer renglon
	double r11 = ((sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6);
    double r12 = -((sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6);
    double r13 = (sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3);
    double px =(_a2*cos(q2) + _d4*cos(q2 + q3))*cos(q1);

    //Segundo renglón
    double r21 = -((sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4))*sin(q6);
    double r22 = ((sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4))*cos(q6);
    double r23 = -(sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3);
    double py = (_a2*cos(q2) + _d4*cos(q2 + q3))*sin(q1);

    //Tercer renglón
    double r31 = -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3);
    double r32 = (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3);
    double r33 = sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5);
    double pz = _a2*sin(q2) + _d1 + _d4*sin(q2 + q3);

    //Generar la matriz de rotación
    Eigen::Matrix<double,4,4> T_0C{{r11,r12,r13,px},
                                   {r21,r22,r33,py},
                                   {r31,r32,r33,pz},
                                   {0,0,0,1}};
    

    return T_0C;


}   

//Función del jacobiano geometrico
Matrix<double,6,6> Robot6DOF::obtenerJacobianoGeometrico(Matrix<double,6,1> Q)
{
    //Obtener los elementos de Q
	double q1 = Q(0);
	double q2 = Q(1);
	double q3 = Q(2);
	double q4 = Q(3);
	double q5 = Q(4);
	double q6 = Q(5);

    //<------------Definir los elementos del jacobiano de la velocidad lineal----------------->

    //Primer renglon
    double Jv11 = -(_a2*cos(q2) + _d4*cos(q2 + q3))*sin(q1);
    double Jv12 = (-_a2*sin(q2) - _d4*sin(q2 + q3))*cos(q1);
    double Jv13 = -_d4*sin(q2 + q3)*cos(q1);

    //Segundo renglón
    double Jv21 = (_a2*cos(q2) + _d4*cos(q2 + q3))*cos(q1);
    double Jv22 = (-_a2*sin(q2) - _d4*sin(q2 + q3))*sin(q1);
    double Jv23 = -_d4*sin(q1)*sin(q2 + q3);

    //Tercer renglón
    double Jv31 = 0;
    double Jv32 = _a2*cos(q2) + _d4*cos(q2 + q3);
    double Jv33 = _d4*cos(q2 + q3);

    //<-------------Definir los elementos del jacobiano de la velocidad angular ------------_>
    //Primer renglón
    double Jw11 = 0;
    double Jw12 = sin(q1);
    double Jw13 = sin(q1);
    double Jw14 = cos(q1)*cos(q2 + q3);
    double Jw15 = sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1);
    double Jw16 = sin(q1)*sin(q4)*sin(q5) - sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) + cos(q1)*cos(q5)*cos(q2 + q3);

    //Segundo renglón
    double Jw21 = 0;
    double Jw22 = -cos(q1);
    double Jw23 = -cos(q1);
    double Jw24 = sin(q1)*cos(q2 + q3);
    double Jw25 = sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4);
    double Jw26 = -sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) + sin(q1)*cos(q5)*cos(q2 + q3) - sin(q4)*sin(q5)*cos(q1);

    //Tercer renglón
    double Jw31 = 1;
    double Jw32 = 0;
    double Jw33 = 0;
    double Jw34 =  sin(q2 + q3);
    double Jw35 =  -sin(q4)*cos(q2 + q3);
    double Jw36 = sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5);
    


    Matrix<double,6,6> jacobianoGeometrico{ {Jv11,Jv12,Jv13,0,0,0},
                                            {Jv21,Jv22,Jv23,0,0,0},
                                            {Jv31,Jv32,Jv33,0,0,0},
                                            {Jw11,Jw12,Jw13,Jw14,Jw15,Jw16},
                                            {Jw21,Jw22,Jw23,Jw24,Jw25,Jw26},
                                            {Jw31,Jw32,Jw33,Jw34,Jw35,Jw36}};
   

}