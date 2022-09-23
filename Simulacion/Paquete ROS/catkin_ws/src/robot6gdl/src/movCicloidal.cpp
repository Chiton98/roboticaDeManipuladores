#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include <string>
#include "gTrayArt.h"

int main(int arc,char **arcv)
{
 
    std::cout<<"Iniciando el nodo\n";
    //Iniciar y crear el nodo deseado
    ros::init(arc,arcv,"state_publisher_robot6GDL");

    //Crear objeto para poder manejar los nodos
    ros::NodeHandle n;
    
    //Objeto que se encargará de realizar la publicación de los joint states
    ros::Publisher joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1);

    //Objeto que se encargar de realizar la publican de los marcadores
    ros::Publisher marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

    int fixed;

    //OBTENER LOS PARÁMETROS del servidor
    n.getParam("/fixed",fixed);

    //Mandar el parametro en el servidor que diga la opción de trayectoria

    //Crear el mensaje de tipo sensor_msgs
    sensor_msgs::JointState msg_joint_state;

    //Crear la forma a mandar
    uint32_t shape = visualization_msgs::Marker::POINTS;


    //Definir el rate del loop
    int rate = 100;

    //Definir la rapidez de iteración en el loop
    ros::Rate loop_rate(rate);

    //Posiciones iniciales del angulo
    double q1 = 0;
    double q2 = 0;
    double q3 = 0;
    double q4 = 0;
    double q5 = 0;
    double q6 = 0;

    //Vector que contiene los nombres de las articulaciones
    std::vector<std::string> joint_names{ "J1","J2","J3","J4","J5","J6" };

    //Generar la trayectoria articular
    
    //Generar el arreglo de tiempo
    double ti = 0;
    double tf = std::stod(arcv[1]);
    double dt = 0.01;
    int numberElements = (tf-ti)/dt;

    //Generar arreglo de tiempo
    Eigen::Array<double,1,Dynamic> arregloTiempo = Eigen::Array<double,1,Dynamic> ::LinSpaced(numberElements+1,ti,tf);

    //Arreglo de trayectorias articulares
    std::cout<<"EMPIEZA LA GENERACIÓN DE TRAYECTORIA CICLOIDAD \n";

    //Variable para almacenar la opcion de la trayectoria
    int opcion;

    //Adquirir la opcion de trayectoria
    n.getParam("/opcion",opcion);
    
    std::tuple<Array3Xd,ArrayXXd> cinematic{generarTrayectoriaPrimitiva(n,arregloTiempo,opcion)};
    
    //Obtener los valores cinematicos

    //Posicion del efector final
    Array3Xd X_0H{std::get<0>(cinematic)};
    
    //VAlores cinematicos
    ArrayXXd Qs{std::get<1>(cinematic)};

    //Variable auxiliar para la iteración   
    int i = 0;

    //Empezar la publicación
    while(ros::ok())
    {
        //Guardar el tiempo en el cual el estado se actualiza
        msg_joint_state.header.stamp = ros::Time::now();

        //CAmbiar el tamaño de los elementos del nombre
        msg_joint_state.name = joint_names;

        //Cambiar el tamaño de los elementos de la posicion
        msg_joint_state.position = std::vector<double> {q1,q2,q3,q4,q5,q6};
        
        //Mandar los angulos al topico
        joint_publisher.publish(msg_joint_state);
	
	if (fixed == 0)
	{
        //Los angulos siempre serán 0
        q1 = 0;
        q2 = 0;
        q3 = 0;
        q4 = 0;
        q5 = 0;
        q6 = 0;
	}
    
    else
    {
        //Actualizar el estado del robot
        q1 = Qs(0,i);
        q2 = Qs(1,i);
        q3 = Qs(2,i);

        //Mandar a publicar la posicion del efector final
        publicarMarker(marker_publisher,X_0H);
        
   
        
        //Reiniciar la variable para seguirnos moviendo
        if (i < Qs.cols()-1 )
        {
            i+=1;
        }
        else
        {
            i = 0;
            std::cout<<"Se reinicia el ciclo\n";
        }

    }

    loop_rate.sleep();
    }
}
