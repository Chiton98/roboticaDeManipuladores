#include <vector>
#include <tuple>
#include "visualization_msgs/Marker.h"
#include "gTrayArt.h"
#include "moduloCinematica.h"

using namespace Eigen;

//Funcion para publicar los valores
void publicarMarker(ros::Publisher &marker_publisher,Array3Xd &X_0H)
{
     //Crear el mensaje
        visualization_msgs::Marker points,linestrip;
        
        points.header.frame_id = linestrip.header.frame_id = "base_link";
        points.header.stamp = linestrip.header.stamp = ros::Time::now();
        points.ns = linestrip.ns = "points_and_lines";
        points.action = linestrip.action = visualization_msgs::Marker::ADD;
        points.id = 0;
        linestrip.id = 1;


        //Definir el tipo en base a una lista de numeros
        points.type = visualization_msgs::Marker::POINTS;
        linestrip.type = visualization_msgs::Marker::LINE_STRIP;

        //Definir la orientación
        points.pose.orientation.w = linestrip.pose.orientation.w = 1.0;

        //Definir la escala
        //PAra el punto
        points.scale.x = 0.001;
        points.scale.y = 0.001;

        //Para la linea que une los puntos
        linestrip.scale.x = 0.002;

        //Puntos son rojos
        points.color.a = 0;
        points.color.r = 1.0;

        //Lineas son cyan
        linestrip.color.a = 1.0;
        linestrip.color.r = 55.0/255.0;
        linestrip.color.g = 0;
        linestrip.color.b = 0;

        for(int i = 0; i < X_0H.cols(); i++)
        {
            
        //Crear geometry msgs
        geometry_msgs::Point p;
        p.x = 0.001*X_0H(0,i) + 0.001;
        p.y = 0.001*X_0H(1,i) + 0.001;
        p.z = 0.001*X_0H(2,i) + 0.001;
        
        //Pushear el point
        points.points.push_back(p);

        linestrip.points.push_back(p);

        }

        
       marker_publisher.publish(points);
       marker_publisher.publish(linestrip);

}

//Función para la trayectoria cubica
Array<double,1,Dynamic> generarTrayectoriaCubica(Array<double,1,Dynamic> &arregloTiempo,Eigen::Vector4d &cE){
    std::cout<<"Generando trayectoria cubica\n";
    //Extraer el tiempo inicial y el tiempo final
    double t0 = arregloTiempo(0);
    double tf = arregloTiempo(Eigen::last);

    
    //Matriz de coeficientes
    Matrix<double,4,4> M{    {1,     t0,     t0*t0,     t0*t0*t0},
                            {0,     1,      2*t0,      3*t0*t0},
                            {1,     tf,     tf*tf,     tf*tf*tf},
                            {0,     1,      2*tf,      3*tf*tf}
                        };
                        
    Matrix<double,4,1> C{   M.inverse()*cE };

    //Arreglo de trayectorias
    ArrayXd Q{ C(0) + C(1)*arregloTiempo + C(2)*arregloTiempo.pow(2) + C(3) * arregloTiempo.pow(3) };

    return Q;
}

//Función para interpolar los polinomios que regresan posicion y velocidad
std::vector<Array<double,1,Dynamic>> generarTrayectoriaCubicaModificada(Eigen::Array<double,1,Dynamic>& arregloTiempo,Eigen::Vector4d &cE)
{
        std::cout<<"Generando trayectoria cubica\n";
    //Extraer el tiempo inicial y el tiempo final
    double t0 = arregloTiempo(0);
    double tf = arregloTiempo(Eigen::last);

    
    //Matriz de coeficientes
    Matrix<double,4,4> M{    {1,     t0,     t0*t0,     t0*t0*t0},
                            {0,     1,      2*t0,      3*t0*t0},
                            {1,     tf,     tf*tf,     tf*tf*tf},
                            {0,     1,      2*tf,      3*tf*tf}
                        };
                        
    Matrix<double,4,1> C{   M.inverse()*cE };


    //Arreglo de trayectorias
    Array<double,1,Dynamic> Q{ C(0) + C(1)*arregloTiempo + C(2)*arregloTiempo.pow(2) + C(3) * arregloTiempo.pow(3) };
    Array<double,1,Dynamic> Qd{C(1) + 2*C(2)*arregloTiempo + 3*C(3) * arregloTiempo.pow(2)};

    std::vector<Array<double,1,Dynamic>> valoresCinematicos;

    valoresCinematicos.push_back(Q);
    valoresCinematicos.push_back(Qd);

    return valoresCinematicos;
}

std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaLineaRecta(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo)
{

    //Generar la trayectoria en linea recta
    std::vector<double> pis;
    std::vector<double> pfs;

    n.getParam("/pi",pis);
    n.getParam("/pf",pfs);

    //Posición inicial
    Vector3d pi{pis[0],pis[1],pis[2]};
    //Posición final
    Vector3d pf{pfs[0],pfs[1],pfs[2]};

    std::cout<<"Generando trayectoria en linea recta\n";  
    
    // <----------------------- DEFINICIÓN DE LA POSICIÓN ----------------------------->
    //Generar array para almacenar las posiciones
    Array3Xd X_0H(3,arregloTiempo.size());

    Vector3d diffp = pf-pi;
	
    //Obtener la longitud de arco final
	double sf = (pf-pi).norm();
    
    //Definir el arreglo de condiciones de estado para la posicion
    Eigen::Vector4d cE{0.0,0.0,sf,0.0};

    //Realizar la interpolación de la longitud de arco s(t)
    std::cout<<"Generando la trayectoria de longitud de arco\n";
    Eigen::Array<double,1,Dynamic> sT{generarTrayectoriaCubica(arregloTiempo,cE)};
	
	//Posicion en X
	Array<double,1,Dynamic> positionX = pi(0) + sT*diffp(0)/sf;
	
	//Posicion en Y
	Array<double,1,Dynamic> positionY = pi(1) + sT*diffp(1)/sf;
	
	//Posicion en Z
	Array<double,1,Dynamic> positionZ = pi(2) + sT*diffp(2)/sf;
	
	//Almacenar los valores en vector X_0H es de 6x1 contiene posición y orientación
	X_0H.row(0) = positionX;
	X_0H.row(1) = positionY;
	X_0H.row(2) = positionZ;
  
     std::cout<<"Generando los angulos de articulación\n";

    //Generar arreglo que contendrá los ángulos
    ArrayXXd Qs(3,arregloTiempo.size());
    
    //Crear el robot
    Robot6DOF instanciaRobot{};

    //Iterar para evaluar los valores de la herramienta
    for(int i = 0; i < arregloTiempo.size(); i++)
    {
        Qs.col(i) = instanciaRobot.obtenerAngulosArticulares(Vector3d{X_0H(0,i),X_0H(1,i),X_0H(2,i)});     
    }    

    //Crear tupla para regresar
    std::tuple<Array3Xd,ArrayXXd> cinematic{X_0H,Qs};

    return cinematic;


}

//Funcion para generar la trayectoria en linea recta de la orientación
std::vector<Eigen::Array<double,6,Dynamic>> generarTrayectoriaLineaRectaOrientacion(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo)
{
    
    //Generar la trayectoria en linea recta
    std::vector<double> pis;
    std::vector<double> pfs;
    std::vector<double> euleris;
    std::vector<double> eulerfs;

    n.getParam("/pi",pis);
    n.getParam("/pf",pfs);
    n.getParam("/euleri",euleris);
    n.getParam("/eulerf",eulerfs);
    

    //Twist inicial
    Vector3d pi{pis[0],pis[1],pis[2]};
    Vector3d euleri{euleris[0],euleris[1],euleris[2]};

    //Twist final
    Vector3d pf{pfs[0],pfs[1],pfs[2]};
    Vector3d eulerf{eulerfs[0],eulerfs[1],eulerfs[2]};

    std::cout<<"Generando trayectoria en linea recta\n";  
    
    // <----------------------- DEFINICIÓN DEL TWIST ----------------------------->

    // <--------------------POSICION--------------------->
    //Generar array para almacenar el TWIST
    Array<double,6,Dynamic> X_0H(6,arregloTiempo.size());

    //Array para generar la razón de cambio del TWIST
    Array<double,6,Dynamic> V_0H(6,arregloTiempo.size());

    Vector3d diffp = pf-pi;
	
    //Obtener la longitud de arco final de la posicion
	double sfp = (pf-pi).norm();
    
    //Definir el arreglo de condiciones de estado para la posicion
    Eigen::Vector4d cEp{0.0,0.0,sfp,0.0};

    //Realizar la interpolación de la longitud de arco s(t)
    std::cout<<"Generando la trayectoria de longitud de arco para la posicion\n";
    std::vector<Eigen::Array<double,1,Dynamic>> sTp{generarTrayectoriaCubicaModificada(arregloTiempo,cEp)};
	
	//Posicion en X
	Array<double,1,Dynamic> positionX = pi(0) + sTp[0]*diffp(0)/sfp;
	
	//Posicion en Y
	Array<double,1,Dynamic> positionY = pi(1) + sTp[0]*diffp(1)/sfp;
	
	//Posicion en Z
	Array<double,1,Dynamic> positionZ = pi(2) + sTp[0]*diffp(2)/sfp;

    //Velocidad en X
	Array<double,1,Dynamic> velocidadX = sTp[1]*diffp(0)/sfp;
    
	//Posicion en Y
	Array<double,1,Dynamic> velocidadY = sTp[1]*diffp(1)/sfp;
	
	//Posicion en Z
	Array<double,1,Dynamic> velocidadZ = sTp[1]*diffp(2)/sfp;
    // <---------------------- ORIENTACIÓN------------------------>

    Vector3d diffo = eulerf-euleri;
	
    //Obtener la longitud de arco final
	double sfo = (eulerf-euleri).norm();
    
    //Definir el arreglo de condiciones de estado para la posicion
    Eigen::Vector4d cEo{0.0,0.0,sfo,0.0};

    //Realizar la interpolación de la longitud de arco s(t)
    std::cout<<"Generando la trayectoria de longitud de arco para la orientacion\n";
    std::vector<Eigen::Array<double,1,Dynamic>> sTo{generarTrayectoriaCubicaModificada(arregloTiempo,cEo)};
	
	//Orientacion en X
	Array<double,1,Dynamic> alfa = euleri(0) + sTo[0]*diffo(0)/sfo;
	
	//Orientacion en Y
	Array<double,1,Dynamic> beta = euleri(1) + sTo[0]*diffo(1)/sfo;

	//Orientacion en Z
	Array<double,1,Dynamic> gamma = euleri(2) + sTo[0]*diffo(2)/sfo;

    //Razón de cambio de la Orientacion en X
	Array<double,1,Dynamic> rateAlfa = sTo[1]*diffo(0)/sfo;
	
	//Razón de cambio de la Orientación en Y
	Array<double,1,Dynamic> rateBeta = sTo[1]*diffo(1)/sfo;
	
	//Razón de cambio de la Orientación en Z
	Array<double,1,Dynamic> rateGamma = sTo[1]*diffo(2)/sfo;
	
	std::cout<<"Almacenando los valores de la posicion\n";
	
	//Almacenar los valores en vector X_0H es de 6x1 contiene posición y orientación
    X_0H.row(0) = positionX;
    X_0H.row(1) = positionY;
    X_0H.row(2) = positionZ;
	X_0H.row(3) = alfa;
	X_0H.row(4) = beta;
	X_0H.row(5) = gamma;
    

  //  Almacenar los valores para la orientación
    V_0H.row(0) = velocidadX;
    V_0H.row(1) = velocidadY;
    V_0H.row(2) = velocidadZ;
	V_0H.row(3) = rateAlfa;
	V_0H.row(4) = rateBeta;
	V_0H.row(5) = rateGamma;
//
    std::cout<<"Generando tray\n";
    //Almacenar los elementos en un std::vector
    std::vector<Eigen::Array<double,6,Dynamic>> tray;

    std::cout<<"Pusheando los valores al vector\n";
    tray.push_back(X_0H);
    tray.push_back(V_0H);
 

    return tray;

}

//Función para generar la trayectoria circular
std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaCircular(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo)
{
    //Parámetros para la trayectoria circular
    std::vector<double> piParam; // Posición inicial dentro de la trayectoria
    std::vector<double> cParam; //Centro del circulo
    std::vector<double> RParam; //Matriz de rotación

    n.getParam("/piC",piParam);
    n.getParam("/c",cParam);
    n.getParam("/R",RParam);

    //Posición inicial en la circunferencia del circulo
    Eigen::Vector3d pi{piParam[0],piParam[1],piParam[2]};
    
    //Centro del círculo
    Eigen::Vector3d c{cParam[0],cParam[1],cParam[2]};

    //Matriz de rotación que proyecta el circulo en su eje local al eje global
    Eigen::Matrix3d R{{RParam[0],RParam[1],RParam[2]},
                      {RParam[3],RParam[4],RParam[5]},
                      {RParam[6],RParam[7],RParam[8]}};

    //Determinar el radio del circulo
    double rho = (pi-c).norm();

    //Definir la longitud de arco final del circulo
    double sf = 2*M_PI*rho;

    //Generar arreglo para almacenar el circulo
    Eigen::Array3Xd pPrime{3,arregloTiempo.size()};

    //Definir las condiciones iniciales
    Eigen::Vector4d cE{0.0,0.0,sf,0.0};

    Eigen::Array<double,1,-1> pS{generarTrayectoriaCubica(arregloTiempo,cE)};

    //Trayectoria en el sistema de referencia local
    pPrime.row(0) = rho*Eigen::sin(pS/rho); 
    pPrime.row(1) = rho*Eigen::cos(pS/rho);
    pPrime.row(2) = 0*Eigen::Array<double,1,-1>::Ones(arregloTiempo.size());
        
    //Generar arreglo para almacenar el circulo
    Eigen::Array3Xd X_0H{3,arregloTiempo.size()};

    //Trayectoria en el sistema de referencia global
    X_0H.row(0) = c(0) + (R*pPrime.matrix()).array().row(0);
    X_0H.row(1) = c(1) + (R*pPrime.matrix()).array().row(1);
    X_0H.row(2) = c(2) + (R*pPrime.matrix()).array().row(2);
    
    //Generar angulos articulares
    Eigen::Array<double,3,Dynamic> Qs(3,arregloTiempo.size());

    //Convertir la trayectoria en espacio cartesiano a espacio articular
    
    //Crear el robot
    Robot6DOF instanciaRobot{};
    std::cout<<"Generando trayectoria en espacio articular\n";

    //Iterar para evaluar los valores de la herramienta
    for(int i = 0; i < arregloTiempo.size(); i++)
    {
        Qs.col(i) = instanciaRobot.obtenerAngulosArticulares(Vector3d{X_0H(0,i),X_0H(1,i),X_0H(2,i)});      
    }    

    //Crear tupla


    return std::tuple<Array3Xd,ArrayXXd>{X_0H,Qs};
}

std::tuple<Array3Xd,ArrayXXd> generarTrayectoriaPrimitiva(ros::NodeHandle &n,Array<double,1,Dynamic> &arregloTiempo,int &opcion)
{
            
    //Generar arreglo para almacenar la trayectoria en espacio cartesiano
    Eigen::Array3Xd X_0H{3,arregloTiempo.size()};
    
    //Generar arreglo para almacenar la trayectoria en espacio articular
    Eigen::Array<double,3,Dynamic> Qs(3,arregloTiempo.size());

    //Se selecciona la trayectoria circular opcion = 0;

    //Se selecciona la trayectoria cicloidal opcion = 1:

    //SE selecciona la trayectoria helocoidal opcion = 2;
   
    if(opcion == 1)
    {
    
    std::cout<<"Trayectoria cicloidal seleccionada\n";
    //Parámetros para la trayectoria circular
    std::vector<double> piParam; // Posición inicial de la cicloide
    double rParam; //Radio del circulo de la cicloide
    std::vector<double> RParam; //Matriz de rotación para proyectar al eje fijo

    n.getParam("/piC",piParam);
    n.getParam("/r",rParam);
    n.getParam("/R",RParam);

    //Vector del vector de posicion inicial
    Eigen::Vector3d pi{piParam[0],piParam[1],piParam[2]};
    
    //Radio del circulo
    double r{rParam};

    //Matriz de rotación que proyecta el circulo en su eje local al eje global
    Eigen::Matrix3d R{{RParam[0],RParam[1],RParam[2]},
                      {RParam[3],RParam[4],RParam[5]},
                      {RParam[6],RParam[7],RParam[8]}};

    //Definir la longitud de arco final de la cicloide
    double sf = 8*r;

    //Generar arreglo para almacenar el circulo
    Eigen::Array3Xd pPrime{3,arregloTiempo.size()};

    //Definir las condiciones iniciales
    Eigen::Vector4d cE{0.0,0.0,sf,0.0};

    Eigen::Array<double,1,-1> s{generarTrayectoriaCubica(arregloTiempo,cE)};
    
    //Trayectoria en el sistema de referencia local
    pPrime.row(0) = r*(2*Eigen::acos((4*r-s)/(4*r)) - Eigen::sin(2*Eigen::acos((4*r-s)/(4*r))) );
    pPrime.row(1) = r*(1 - Eigen::cos(2*Eigen::acos( (4*r-s) / (4*r))) );
    pPrime.row(2) = 0*Eigen::Array<double,1,-1>::Ones(arregloTiempo.size());

    //Trayectoria en el sistema de referencia global
    X_0H.row(0) = pi(0) + (R*pPrime.matrix()).array().row(0); //Componente en X
    X_0H.row(1) = pi(1) + (R*pPrime.matrix()).array().row(1); //Componente en Y
    X_0H.row(2) = pi(2) + (R*pPrime.matrix()).array().row(2); //Componente en Z

    //Convertir la trayectoria en espacio cartesiano a espacio articular
 
    //Crear el robot
    Robot6DOF instanciaRobot{};
    std::cout<<"Generando trayectoria en espacio articular\n";

    //Iterar para evaluar los valores de la herramienta
    for(int i = 0; i < arregloTiempo.size(); i++)
    {
        Qs.col(i) = instanciaRobot.obtenerAngulosArticulares(Vector3d{X_0H(0,i),X_0H(1,i),X_0H(2,i)});      
    }    
     return std::tuple<Array3Xd,ArrayXXd>{X_0H,Qs};
    }
   
    //Trayectoria cicloidal
    else if (opcion == 2)
    {
        std::cout<<"Trayectoria helocoidal seleccionada\n";
        //Parámetros para la trayectoria circular
        std::vector<double> piParam; // Posición inicial de la helice
        double rParam; //Radio del circulo de la cicloide
        std::vector<double> RParam; //Matriz de rotación para proyectar al eje fijo

        double aParam;
        n.getParam("/piC",piParam);
        n.getParam("/r",rParam);
        n.getParam("/R",RParam);
        n.getParam("/a",aParam);

        //Vector del vector de posicion inicial
        Eigen::Vector3d pi{piParam[0],piParam[1],piParam[2]};
        
        //Radio del circulo
        double r{rParam};

        //Razon de cambio de la coordenada z
        double a{aParam};

        //Matriz de rotación que proyecta el circulo en su eje local al eje global
        Eigen::Matrix3d R{{RParam[0],RParam[1],RParam[2]},
                        {RParam[3],RParam[4],RParam[5]},
                        {RParam[6],RParam[7],RParam[8]}};

        //Definir la longitud de arco final de la cicloide
        double sf = 2*sqrt(2)*M_PI;

        //Generar arreglo para almacenar el circulo
        Eigen::Array3Xd pPrime{3,arregloTiempo.size()};

        //Definir las condiciones iniciales
        Eigen::Vector4d cE{0.0,0.0,sf,0.0};

        Eigen::Array<double,1,-1> s{generarTrayectoriaCubica(arregloTiempo,cE)};
        
        //Trayectoria en el sistema de referencia local
        pPrime.row(0) = r*Eigen::cos(s/sqrt(r*r+a*a));
        pPrime.row(1) = r*Eigen::sin(s/sqrt(r*r+a*a));
        pPrime.row(2) = r*s/(sqrt(r*r+a*a))*Eigen::Array<double,1,-1>::Ones(arregloTiempo.size());

        //Trayectoria en el sistema de referencia global
        X_0H.row(0) = pi(0) + (R*pPrime.matrix()).array().row(0); //Componente en X
        X_0H.row(1) = pi(1) + (R*pPrime.matrix()).array().row(1); //Componente en Y
        X_0H.row(2) = pi(2) + (R*pPrime.matrix()).array().row(2); //Componente en Z

        //Convertir la trayectoria en espacio cartesiano a espacio articular
    
        //Crear el robot
        Robot6DOF instanciaRobot{};
        std::cout<<"Generando trayectoria en espacio articular\n";

        //Iterar para evaluar los valores de la herramienta
        for(int i = 0; i < arregloTiempo.size(); i++)
        {
            Qs.col(i) = instanciaRobot.obtenerAngulosArticulares(Vector3d{X_0H(0,i),X_0H(1,i),X_0H(2,i)});      
        }    
        return std::tuple<Array3Xd,ArrayXXd>{X_0H,Qs};
    }
    
    //Trayectoria aún no implementada
    else
    {
        std::cout<<"Aún no lo implementamos";
        std::tuple<Array3Xd,ArrayXXd> a;
        return a;
    }
  
}