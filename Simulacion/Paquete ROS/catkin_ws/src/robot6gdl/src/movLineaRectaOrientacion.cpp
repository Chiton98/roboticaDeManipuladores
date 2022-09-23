#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include <string>
#include "gTrayArt.h"


int main(int arc,char **arcv)
{
 
    std::cout<<"Iniciando el nodo\n";
    //Iniciar y crear el nodo deseado
    ros::init(arc,arcv,"state_publisher_robot6GDL");

    //Definir grados de libertad del robot
    int gdl = 6;

    //Crear objeto para poder manejar los nodos
    ros::NodeHandle n;
    
    //Objeto que se encargará de realizar la publicación
    ros::Publisher joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1);


    //OBTENER LOS PARÁMETROS del servidor
    int fixed;
    n.getParam("/fixed",fixed);

    //Crear el mensaje de tipo sensor_msgs
    sensor_msgs::JointState msg_joint_state;

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
    
    //Generar el arreglo de tiempo
    double ti = 0;
    double tf = std::stod(arcv[1]);;
    double dt = 0.01;

    int numberElements = (tf-ti)/dt;

    //Generar arreglo de tiempo
    Eigen::Array<double,1,Dynamic> arregloTiempo = Eigen::Array<double,1,Dynamic> ::LinSpaced(numberElements,ti,tf);

    //Generar trayectoria linea recta

    std::vector<Eigen::Array<double,6,Dynamic>> tray{generarTrayectoriaLineaRectaOrientacion(n,arregloTiempo)};
    
   //Obtener la trayectoria en espacio articular usando RMRC
   std::cout<<"Generando la trayectoria usando RMRC\n";

  


    //Variable auxiliar para la iteración
    //int i = 0;

    std::cout<<"Empezando a publicar posición\n";
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

        loop_rate.sleep();

    }
}
