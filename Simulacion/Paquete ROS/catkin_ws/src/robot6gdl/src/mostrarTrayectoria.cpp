#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc,char** argv)
{
    //Iniciar ros
    ros::init(argc,argv,"mostrar_Trayectoria");

    //Nodo para iniciar todo
    ros::NodeHandle n;
       
    //Duración del rate
    int rate = 5;

     //Definir la rapidez de iteración en el loop
    ros::Rate loop_rate(rate);

    //Nodo
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);



    while(ros::ok())
    {

        //Crear el mensaje
        visualization_msgs::Marker points,linestrip;
        
        points.header.frame_id = linestrip.header.frame_id = "base_link";
        points.header.stamp = linestrip.header.stamp = ros::Time::now();

        

        marker.ns = "cube";
        marker.id = 0;

        //Definir el tipo en base a una lista de numeros
        marker.type = shape;
        //Definir la accion. Añadir
        marker.action = visualization_msgs::Marker::ADD;

        //Actualizar la posicion con los valores del efector final
        marker.pose.position.x = 1;
        marker.pose.position.y = 0;
        marker.pose.position.z = 2;

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;  

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0;


        marker.lifetime = ros::Duration(5);
        
        //Publicar el mensaje
        std::cout<<"Publicando el mensaje\n";
        marker_pub.publish(marker);

        loop_rate.sleep();

            

    }
}