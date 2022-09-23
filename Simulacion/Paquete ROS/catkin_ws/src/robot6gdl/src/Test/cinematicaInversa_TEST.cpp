#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include <string>
#include "claseRobots.h"


int main(int arc,char **arcv)
{
 
    std::cout<<"Iniciando el nodo\n";

    //Capturar la posición deseada
    double x0 = std::stod(arcv[1]);
    double y0 = std::stod(arcv[2]);
    double z0 = std::stod(arcv[3]);

    //Iniciar y crear el nodo deseado
    ros::init(arc,arcv,"state_publisher_robot6GDL");

    //Crear objeto para poder manejar los nodos
    ros::NodeHandle n;
    
    //Objeto que se encargará de realizar la publicación
    ros::Publisher joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1);

    std::vector<double> pi;
    std::vector<double> pf;

    //Crear el mensaje de tipo sensor_msgs
    sensor_msgs::JointState msg_joint_state;

    //Objeto tf
    //tf::TransformBroadcaster broadcaster;

    //Definir el rate del loop
    int rate = 100;
    //Definir la rapidez de iteración en el loop
    ros::Rate loop_rate(rate);

    //Definir grado
    const double degree = M_PI/180;

    //Definir la posicion inicial
    //Generar instancia del robot
    Robot6DOF R{};

    //Definir la posición deseada
    Eigen::Vector3d P_0H{x0,y0,z0};

    //Orientación deseada
    Eigen::Matrix3d R_0H{{0.0,0.0,1.0},
                         {0.0,-1.0,0.0},
                         {1.0,0.0,0.0}};

    Eigen::Array<double,6,1> Qs{R.obtenerAngulosArticulares(P_0H,R_0H)};

    //Posiciones iniciales del angulo
    double q1 = Qs(0,0);
    double q2 = Qs(1,0);
    double q3 = Qs(2,0);

    double q4 = Qs(3,0);
    double q5 = Qs(4,0);
    double q6 = Qs(5,0);
    

    std::cout<<"Los angulos articulares son:\n"<<Qs;

    //Vector que contiene los nombres de las articulaciones
    std::vector<std::string> joint_names{ "J1","J2","J3","J4","J5","J6" };
        
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
