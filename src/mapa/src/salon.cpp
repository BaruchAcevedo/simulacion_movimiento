#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <string.h>
#include <vector>
#include <iostream>
//#include <tf/transform_brodcast.h>
//#include <nav_msgs/Odometry.h>


const int WIDTH=10;
const int HEIGHT=20;
const int WIDTHC = 11;
const int HEIGHTC = 21;


char* mapa= new char[WIDTHC*HEIGHTC];

/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
void fillRectangle(int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++)
  {
    for(int j = j1; j <= j2; j++)
    {
      mapa[i*WIDTHC+j] = value;
    }
  }
}

class obstaculos
{
private:
  int coordenada_x;
  int coordenada_y;
  int coordenada_z;
  double escala_x;
  double escala_y;
  double escala_z;
  int id;
  std::string frame;
  int tipo_obstaculo; //Con tipo 0 una pared, tipo 1 una mesa y tipo 2 una silla
  visualization_msgs::Marker obstaculo;
  ros::Publisher publicador;


public:
  obstaculos(int x, int y, int z, double e_x,double e_y, double e_z, int iden, 
                        std::string sframe, int tipo, ros::Publisher marker_pub);
  void publicar();
};

obstaculos::obstaculos(int x, int y, int z, double e_x,double e_y, double e_z, int iden, 
                        std::string sframe, int tipo, ros::Publisher marker_pub)
{
  publicador = marker_pub;
  coordenada_x = x;
  coordenada_y = y;
  coordenada_z = z;
  escala_x = e_x;
  escala_y = e_y;
  escala_z = e_z;
  int inicio_y=(y-((int)e_y)/2)+(HEIGHTC/2);
  int inicio_x=(x-((int)e_x)/2)+(WIDTHC/2);
  int final_y=(y+((int)e_y)/2)+(HEIGHTC/2);
  int final_x=(x+((int)e_x)/2)+(WIDTHC/2);

  fillRectangle(inicio_y,inicio_x,final_y,final_x,100);

  id = iden;
  frame = sframe;
  obstaculo.header.frame_id = sframe;
  obstaculo.header.stamp = ros::Time::now();
  obstaculo.ns = std::to_string(iden);
  obstaculo.id = iden;
  tipo_obstaculo = tipo;
  if (tipo == 0 || tipo == 1 || tipo == 3) {
    obstaculo.type = visualization_msgs::Marker::CUBE;
  }
  else {
    obstaculo.type = visualization_msgs::Marker::CYLINDER;
  }
  obstaculo.action = visualization_msgs::Marker::ADD;

  obstaculo.pose.position.x = coordenada_x;
  obstaculo.pose.position.y = coordenada_y;
  obstaculo.pose.position.z = coordenada_z;
  obstaculo.pose.orientation.x = 0.0;
  obstaculo.pose.orientation.y = 0.0;
  obstaculo.pose.orientation.z = 0.0;
  obstaculo.pose.orientation.w = 1.0;
  obstaculo.scale.x = escala_x;
  obstaculo.scale.y = escala_y;
  obstaculo.scale.z = escala_z;
  if(tipo!=3){  
    obstaculo.color.r = 0.0f;
    obstaculo.color.g = 1.0f;
    obstaculo.color.b = 0.0f;
    obstaculo.color.a = 1.0;
  }else{
    obstaculo.color.r = 0.0f;
    obstaculo.color.g = 1.0f;
    obstaculo.color.b = 1.0f;
    obstaculo.color.a = 1.0;
  }
  obstaculo.lifetime = ros::Duration();
}

void obstaculos::publicar(){
  publicador.publish(obstaculo);
  return;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "mapa");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);
  nav_msgs::OccupancyGrid map;

  std::string frame = "/odom";
  int posicion_x=0;
  int posicion_y=0;
  int destino_x=10;
  int destino_y=10;

  map.header.frame_id = frame;
  map.header.stamp = ros::Time::now();   // No caduca

  map.info.resolution = 1.0;             // [m/cell]
  map.info.width = WIDTHC;                // [cells]
  map.info.height = HEIGHTC;              // [cells]
  map.info.origin.position.x = -WIDTHC/2-0.5;
  map.info.origin.position.y = -HEIGHTC/2-0.5;
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;

  /*Incializa los valores de la matriz mapa en 0*/

  fillRectangle(0,0,HEIGHT,WIDTH,0);
  
  //imprime_mapa();

  /*Crea los obstaculos*/
  /*                            x,y,z,e_x,e_y,e_z,iden,frame,tipo,marker*/
  obstaculos pared1 = obstaculos(5,0,2,1.0,20.0,5.0,0,frame,0,marker_pub);
  obstaculos pared2 = obstaculos(0,10,2,10.0,1.0,5.0,1,frame,0,marker_pub);
  obstaculos pared3 = obstaculos(0,-10,2,10.0,1.0,5.0,2,frame,0,marker_pub);
  obstaculos pared4 = obstaculos(-5,0,2,1.0,20.0,5.0,3,frame,0,marker_pub);
  obstaculos mesa1 = obstaculos(2,4,0,2.0,1.0,1.0,4,frame,1,marker_pub);
  obstaculos mesa2 = obstaculos(2,2,0,2.0,1.0,1.0,5,frame,1,marker_pub);
  obstaculos mesa3 = obstaculos(2,-2,0,2.0,1.0,1.0,6,frame,1,marker_pub);
  obstaculos mesa4 = obstaculos(-2,2,0,2.0,1.0,1.0,7,frame,1,marker_pub);
  obstaculos mesa5 = obstaculos(-2,-2,0,2.0,1.0,1.0,8,frame,1,marker_pub);
  obstaculos mesa6 = obstaculos(-2,-4,0,2.0,1.0,1.0,9,frame,1,marker_pub);
  obstaculos mesa7 = obstaculos(2,-4,0,2.0,1.0,1.0,10,frame,1,marker_pub);

  map.data = std::vector<int8_t>(mapa, mapa + (WIDTHC*HEIGHTC));
  while (ros::ok())
  {
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Publica los obstaculos
    pared1.publicar();
    pared2.publicar();
    pared3.publicar();
    pared4.publicar();
    mesa1.publicar();
    mesa2.publicar();
    mesa3.publicar();
    mesa4.publicar();
    mesa5.publicar();    
    mesa6.publicar();
    mesa7.publicar();
    //suelo.publicar();

    occupancy_pub.publish(map);

    r.sleep();
  }
}