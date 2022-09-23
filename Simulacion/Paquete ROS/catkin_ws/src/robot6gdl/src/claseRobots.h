#ifndef _CLASE_ROBOTS_H
#define _CLASE_ROBOTS_h

#include <eigen3/Eigen/Dense>
using namespace Eigen;

//Definir la clase robot como abstracta
class Robot
{
    public:
        //Constructor
        Robot()
        {};
        //Destructor 
        virtual ~Robot(){};
 
 	//Función de cinemática directa
 	virtual Matrix<double,4,4> obtenerCinematicaDirecta(Matrix<double,6,1> &Q)=0;
 	
	//Función de cinemática inversa
        virtual Array<double,3,1> obtenerAngulosArticulares(Vector3d P_0H)=0;
        
        //Función jacobiana
        virtual Matrix<double,6,6> obtenerJacobianoGeometrico(Matrix<double,6,1> Q)=0;
        
};

//Definir la clase para el robot de 6GDL
class Robot6DOF:public Robot

{
    public: 
        Robot6DOF()
        {
            _d1 = 53.0;
            _a2 = 220.0;
            _d4 = 200.0;
            _d6 = 114.0;
            _a3 = _d4+_d6;
            T_CH = Matrix4d{ {1.0,0.0,0.0,0.0},
                              {0.0,1.0,0.0,0.0},
                              {0.0,0.0,1.0,_d6},
                              {0.0,0.0,0.0,1.0}};
        }
    
    	//Redefinir la función de cinemática directa
    	virtual Matrix<double,4,4> obtenerCinematicaDirecta(Matrix<double,6,1> &Q) override;
    	
        //Redefinir la funcion virtual obtener angulos
        virtual Array<double,3,1> obtenerAngulosArticulares(Vector3d P_0H) override;
	
	    //Redefinir la función de jacobiano geometrico
	    virtual Matrix<double,6,6> obtenerJacobianoGeometrico(Matrix<double,6,1> Q) override;

    
    private:
        Matrix4d T_CH; //Matriz de transformación homogénea de la herramienta respecto a al centro de la muñeca
        double _d1;
        double _a2;
        double _d4;
        double _d6;
        double _a3;
};
#endif
