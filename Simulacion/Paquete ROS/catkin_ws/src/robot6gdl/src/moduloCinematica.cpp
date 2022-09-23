#include "moduloCinematica.h"

//Función para convertir de parametros de euler a Rotation matrix
Eigen::Matrix3d eulerZYXToRotationMatrix(double alfa,double beta,double gamma)
{
   
    //Primer renglon
    double r11 = cos(alfa)*cos(beta);
    double r12 = cos(alfa)*sin(beta)*sin(gamma)-sin(alfa)*cos(gamma);
    double r13 = cos(alfa)*sin(beta)*cos(gamma)+ sin(alfa)*sin(gamma);

    //Segundo renglón
    double r21 = sin(alfa)*cos(beta);
    double r22 = sin(alfa)*sin(beta)*sin(gamma)+cos(alfa)*cos(gamma);
    double r23 = sin(alfa)*sin(beta)*cos(gamma)- cos(alfa)*sin(gamma);

    //Tercer renglón
    double r31 = -sin(beta);
    double r32 = cos(beta)*sin(gamma);
    double r33 = cos(beta)*cos(gamma);

    Eigen::Matrix3d R { {r11,r12,r13},
                        {r21,r22,r23},
                        {r31,r32,r33}};

    return R;
}

//Funcion para obtener la matriz E, convierte razon de cambio de alfa,beta,gamm
//a velocidades angulares wx,wy,wz
Eigen::Matrix3d obtenerMatrizE(double alfa,double beta,double gamma){

    Eigen::Matrix3d E{{0,-sin(alfa),cos(beta)*cos(alfa)},
                      {0,cos(alfa),cos(beta)*sin(alfa)},
                      {1,0,-sin(beta)}};

    return E;

}

Eigen::Vector3d rotationMatrixtoeulerZYX(Eigen::Matrix3d R)
{

    //Evaluar los angulos
    double alfa;
    double beta;
    double gamma;
    
    //Beta (angulo de rotación en Y)
    double beta = atan2(-R(2,0),hypot(R(0,0),R(1,0)));

    //Comprobar el valor de beta
    if (beta == M_PI/2 or beta == -M_PI/2)
    {
       if (beta ==  M_PI/2)
        {
            beta =  M_PI/2;
            alfa = 0.0;
            gamma =atan2(R(0,1),R(1,1));
        }
           
        else{
            beta = -M_PI/2;
            alfa = 0.0;
            gamma = -atan2(R(0,1),R(1,1));
        }
            
    }
    
    //LA solución no se degenera
    else
    {
        //Gamma(angulo de rotación en Z)
        alfa = atan2(R(1,0)/cos(beta),R(0,0)/cos(beta));

        //Alpha(angulo de rotación en X)
        gamma = atan2(R(2,1)/cos(beta),R(2,2)/cos(beta));
    }
    
    //Empaquetar los valores
    Eigen::Vector3d params{alfa,beta,gamma}; //Regresa angulo en : (Z,Y,X)

    return params;
}