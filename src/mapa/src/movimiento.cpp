#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

const int SENSORES = 6;

const int WIDTHC = 11;
const int HEIGHTC = 21;

const float PI = 3.1416;

const float resolution = 1.0;

int x_global=(WIDTHC/2);
int y_global=(HEIGHTC/2);

float direcciones[SENSORES]; //Donde se encuentran, para cada uno de los 8 vectores, los angulos de los vectores, de los cuales se 
                            // calculan las rectas dadá las coordenadas del robot y estos angulos.

float magnitudes[SENSORES];

float direccion[2]; // vector que determina a donde se dirige. Dadas las coordenadas globales.

float objetivo[2]; // Vector que determina a donde se tiene que mover el robot.

float VELOCIDAD_MAXIMA = 0.1; // Velocidad máxima a la cual se moverá el robot, en caso de excederla, tomará esta.

float x_odom=0.0;
float y_odom=0.0;
float angle_odom=PI;
float vx_odom = 0.0;
float vz_odom = 0.0;

/**
 * Función auxiliar que permite, desde un cuaternion, conseguir la tranformación a euler. Sobre el eje z.
 * @param q Quaternion desde el cual se transformará.
 * @return Transformación a Euler.
 */
double to_euler(geometry_msgs::Quaternion q){
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return atan2(siny_cosp,cosy_cosp);
}

/**
 * Función que da como resultado el angulo entre dos vectores.
 * @param x1 Posción del vector 1 en x.
 * @param y1 Posción del vector 1 en y.
 * @param x2 Posción del vector 2 en x.
 * @param y2 Posción del vector 2 en y.
 * @return Angulo resultante, 0 si alguno de los vectores es 0.
 */
float angulo(float x1, float y1, float x2, float y2){
  float mod1 = sqrt( pow(x1,2) + pow(y1,2) );
  float mod2 = sqrt( pow(x2,2) + pow(y2,2) );
  float point = x1*x2 + y1*y2;
  if(mod1 == 0 || mod2 == 0){
    return 0;
  }else{
    return acos(point / (mod1*mod2));
  }
}

/**
 * Función que da como resultado el valor en y de un una recta, respecto a una x.
 * @param theta Angulo de la recta.
 * @param x1 Valor en x de un punto de la recta.
 * @param y1 Valor en y de un punto de la recta.
 * @param x Valor del cual se desea saber el resultado de la función de la recta.
 * @return Valor en y respecto a está recta. 
 */
float recta(float theta, float x1, float y1, float x){
  return tan(theta)*(x-x1)+y1;
}

/** 
 * Función que encuentra la colision más cercana. Considera que tras el mapa todo son coliciones.
 * Considera que theta es 0<theta<2PI
 * @param x Posición del robot en x.
 * @param y Posición del robot en y.
 * @param theta Angulo del sensor que se medirá.
 * @param mapa Mapa del mundo en el cual se mueve el robot.
 * @return Valor de la distancia a la cual se encuentra un obstaculo.
*/
float encuentraColision(float x, float y, float theta, const signed char* mapa){
  int y_mapa=(int)y+(HEIGHTC/2);
  int x_mapa=(int)x+(WIDTHC/2);
  int y_robot=(int)y+(HEIGHTC/2);
  int x_robot=(int)x+(WIDTHC/2);


  //ROS_INFO("Posicion actual en el mapa: %d %d",y_mapa,x_mapa);

  if(theta == PI/2.0){
    while(mapa[y_mapa*WIDTHC+x_mapa]!=100 && y_mapa >0){
      y_mapa++;
    }
  }
  if(theta == (3.0*PI)/2.0){
    while(mapa[y_mapa*WIDTHC+x_mapa]!=100 && y_mapa <HEIGHTC){
      y_mapa--;
    }
  }
  if(theta < PI/2.0 || theta > (3.0*PI)/2.0){
    while(x_mapa < WIDTHC && mapa[y_mapa*WIDTHC+x_mapa]!=100 && y_mapa<HEIGHTC && y_mapa>0){
      x_mapa++;
      float y_objetivo = recta(theta,x,y,(x_mapa-(WIDTHC/2.0)));
      int y_m_objetivo =((int)y_objetivo+(HEIGHTC/2)); 
      if(y_m_objetivo>y_mapa){
        while(y_mapa!=y_m_objetivo){
          y_mapa++;
          if(mapa[y_mapa*WIDTHC+(x_mapa-1)]==100 || y_mapa>HEIGHTC){
            x_mapa--;
            break;
          }
        }
      }else{
        while(y_mapa!=y_m_objetivo){
          y_mapa--;
          if(mapa[y_mapa*WIDTHC+(x_mapa-1)]==100 || y_mapa<0){
            x_mapa--;
            break;
          }
        }
      }
    }
  }else{
    while(x_mapa > 0 && mapa[y_mapa*WIDTHC+x_mapa]!=100 && y_mapa<HEIGHTC && y_mapa>0){
      x_mapa--;
      float y_objetivo = recta(theta,x,y,(x_mapa-(WIDTHC/2.0)));
      int y_m_objetivo =((int)y_objetivo+(HEIGHTC/2)); 
      if(y_m_objetivo>y_mapa){
        while(y_mapa!=y_m_objetivo){
          y_mapa++;
          if(mapa[y_mapa*WIDTHC+(x_mapa-1)]==100||y_mapa>HEIGHTC){
            x_mapa++;
            break;
          }
        }
      }else{
        while(y_mapa!=y_m_objetivo){
          y_mapa--;
          if(mapa[y_mapa*WIDTHC+(x_mapa-1)]==100 || y_mapa<0){
            x_mapa++;
            break;
          }
        }
      }
    }
  }

  float p_x = pow(((float)x_mapa-((float)WIDTHC/2.0)-x),2);
  float p_y = pow(((float)y_mapa-((float)HEIGHTC/2.0)-y),2);
  float resultado = sqrt( p_x + p_y ); 
  return resultado;

}

/**
 * Publica una nueva velocidad al robot, evitando que pasé de la velocidad maxima.
 * @param vx Velocidad lineal del robot.
 * @param vth Velocidad angular del robot.
 * @param velocity_pub Publicador al cual se enviarán los datos del robot.
 */
void publicar(float vx, float vth, ros::Publisher velocity_pub){
    //Ulitima actualizacion
    if(vx > VELOCIDAD_MAXIMA)
      vx = VELOCIDAD_MAXIMA;
    if(vx < -VELOCIDAD_MAXIMA)
      vx = -VELOCIDAD_MAXIMA;
    if(vth > VELOCIDAD_MAXIMA)
      vth = VELOCIDAD_MAXIMA;
    if(vth < -VELOCIDAD_MAXIMA)
      vth = -VELOCIDAD_MAXIMA;

    geometry_msgs::Twist velocidad;

    velocidad.angular.z=vth;
    velocidad.linear.x=vx;

	  velocidad.linear.y = 0;
	  velocidad.linear.z = 0;
	  velocidad.angular.x = 0;
	  velocidad.angular.y = 0;

    velocity_pub.publish(velocidad);
}

/**
 * Función que calcula las magnitudes de los obstaculos respecto al robot, guardandolo en la 
 * variable global magnitudes.
 * @param grid Mapa en el cual se mueve el robot.
 */
void encuentraCoodenadas(const nav_msgs::OccupancyGrid grid){
  const signed char* data = grid.data.data();
  for(int i = 0; i < SENSORES; i++)
  {
    magnitudes[i] = encuentraColision(x_odom,y_odom,direcciones[i],data);
  }
}

/**
 * Función que cambia los datos guardados del robot respecto al odometro.
 * @param odom Datos del odometro del robot.
 */
void actualiza(nav_msgs::Odometry odom){
  x_odom = odom.pose.pose.position.x;
  y_odom = odom.pose.pose.position.y;
  // Dejo estas lineas por si es más útil que la función to_euler
  //tf2::Quaternion q = tf2::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
  //angle_odom = q.getAngle();
  angle_odom = to_euler(odom.pose.pose.orientation);
  vx_odom = odom.twist.twist.linear.x;
  vz_odom = odom.twist.twist.angular.z;
  y_global=(int)y_odom+(HEIGHTC/2);
  x_global=(int)x_odom+(WIDTHC/2);
  float avance = (angle_odom-PI) - direcciones[0];

  for (int i = 0; i < SENSORES; i++)
  {
    direcciones[i]+=avance;
    if(direcciones[i]>2*PI){
      direcciones[i]-=2*PI;
    }
    if(direcciones[i]<0){
      direcciones[i]+=2*PI;
    }
  }

}

/**
 * Función que modifica el vector al cual se dirige el robot.
 * @param point Punto al cual se dirigirá el robot.
 */
void actualiza_objetivo(geometry_msgs::PointStamped point){
  ROS_INFO("Actualizando objetivo: (%f,%f)",objetivo[0],objetivo[1]);
  objetivo[0]=point.point.y;
  objetivo[1]=point.point.x;
  ROS_INFO("Nuevo objetivo objetivo: (%f,%f)",objetivo[0],objetivo[1]);
}

class flecha
{
private:
  visualization_msgs::Marker vector;
public:
  flecha(float x_1, float y_1, float x_2, float y_2,int id,double r, double g, double b);
  void publicar(ros::Publisher marker_pub);
};

flecha::flecha(float x_1, float y_1, float x_2, float y_2,int id,double r, double g, double b)
{
  vector.header.frame_id = "/odom";
  vector.header.stamp = ros::Time::now();

  vector.id = id;
  vector.ns = std::to_string(id);
  vector.type = visualization_msgs::Marker::ARROW;
  vector.scale.x=0.1;
  vector.scale.y=0.1;
  vector.color.b = b;
  vector.color.r = r;
  vector.color.g = g;
  vector.color.a = 0.5;
  
  geometry_msgs::Point p;
  p.y=y_2;
  p.x=x_2;
  geometry_msgs::Point q;
  q.y=y_1;
  q.x=x_1;
  vector.points.clear();
  vector.points.push_back(q);
  vector.points.push_back(p);

}
void flecha::publicar(ros::Publisher marker_pub){
  marker_pub.publish(vector);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("/occupancy_marker", 5, encuentraCoodenadas);
  ros::Subscriber sub_odom = n.subscribe("/odom", 1, actualiza);
  ros::Subscriber sub_point = n.subscribe("/clicked_point",1,actualiza_objetivo);
  //tf::TransformBroadcaster odom_broadcaster;

  for(int i = 0; i < SENSORES; i++)
  {
    direcciones[i]=((2.0*PI)/(float)SENSORES)*i;
    if(direcciones[i]+angle_odom>2*PI){
      direcciones[i]+=angle_odom-2*PI;
    }else if(direcciones[i]+angle_odom<0){
      direcciones[i]+=angle_odom+2*PI;
    }else
      direcciones[i]+=angle_odom;
  }

  for(int i = 0; i < SENSORES; i++)
  {
    magnitudes[i]=0.0;
  }

  y_global=(int)y_odom+(HEIGHTC/2);
  x_global=(int)x_odom+(WIDTHC/2);


  direccion[0]=0.0;
  direccion[1]=0.0;

  objetivo[0]=0.0;
  objetivo[1]=0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1);
  ROS_INFO("Inicializado");

  while(n.ok()){
/*    
    if((int)objetivo[0]+(HEIGHTC/2)==y_global && (int)objetivo[1]+(WIDTHC/2)==x_global){
      ROS_INFO("He llegado a la meta");
      r.sleep();
    }
*/
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double vf_x;
    double vf_y;
    double delta_th;

    direccion[0]=0.0;
    direccion[1]=0.0;

    //printf("\n");

    for(int i = 0; i < SENSORES; i++)
    {
        direccion[0]=direccion[0]-(1/magnitudes[i])*sin(direcciones[i]);
        direccion[1]=direccion[1]-(1/magnitudes[i])*cos(direcciones[i]);
        flecha(x_odom,y_odom,(magnitudes[i]*cos(direcciones[i]))+x_odom,(magnitudes[i]*sin(direcciones[i]))+y_odom,60+i,1.0,0.5,0.8).publicar(marker_pub);
        //printf("%f,%f,%f,%f\n",x_odom,y_odom,magnitudes[i]*cos(direcciones[i]),magnitudes[i]*sin(direcciones[i]));
    }

    float angulo = angle_odom;

    flecha(x_odom,y_odom,direccion[1]+x_odom,direccion[0]+y_odom,51,1.0,0.7,0.3).publicar(marker_pub);
    flecha(x_odom,y_odom,objetivo[1],objetivo[0],52,0.1,0.3,0.5).publicar(marker_pub);

    //ROS_INFO("Vector aceleracion Actual: (%f,%f)",direccion[0],direccion[1]);

    direccion[0]=(direccion[0])+(objetivo[0]-y_odom);
    direccion[1]=(direccion[1])+(objetivo[1]-x_odom);

    flecha(x_odom,y_odom,direccion[1]+x_odom,direccion[0]+y_odom,50,1.0,0.0,0.0).publicar(marker_pub);

    //printf("%f\n",mod);
    //ROS_INFO("Vector  = : (%f,%f,%f)",y_odom,x_odom,angulo);

    //ROS_INFO("Vector aceleracion Actual: (%f,%f)",direccion[0],direccion[1]);

    //ROS_INFO("Anterior: lineal: %f Angular: %f",velocidad_anterior,velocidad_angular_anterior);

    //ROS_INFO("Angulo actual %f Seno %f Coseno %f",angulo,sin(angulo),cos(angulo));

    double ax = cos(angle_odom)*direccion[1]+sin(angle_odom)*direccion[0];
    double ay =-sin(angle_odom)*direccion[0]+cos(angle_odom)*direccion[1];

    //flecha(x_odom,y_odom,ax+x_odom,ay+y_odom,54,0.9,0.7,0.0).publicar(marker_pub);

    double angular=vz_odom+ay*dt;
    double lineal=vx_odom+ax*dt;
//    vf_y=direccion[0]*dt;
//    vf_x=direccion[1]*dt;
    //ROS_INFO("Vector velocidad calculado: (%f,%f)",vf_y,vf_x);


    //printf("lineal: %f angular: %f\n",lineal,angular);


    //ROS_INFO("Velocidades calculadas: lineal: %f angular: %f",lineal, angular);

    //ROS_INFO("Velocidades calculadas: Lineal: %f ",lineal);
    //ROS_INFO("Angular: %f \n\n",angular);
    //flecha(0.0,0.0,0.0,1.0,71,0.0,0.0,0.0).publicar(marker_pub);
    //flecha(0.0,0.0,1.0,0.0,72,1.0,1.0,1.0).publicar(marker_pub);
    
    //flecha(x_odom,y_odom,cos(angle_odom)+x_odom,sin(angle_odom)+y_odom,52,0.0,1.0,0.0).publicar(marker_pub); //El x
    //flecha(x_odom,y_odom,-sin(angle_odom)+x_odom,cos(angle_odom)+y_odom,53,0.5,1.0,0.5).publicar(marker_pub); //En y
    //flecha(x_odom,y_odom,lineal+x_odom,angular+y_odom,51,0.0,0.0,1.0).publicar(marker_pub);
    //flecha(0,0,vx_odom,vz_odom,54).publicar(marker_pub);

/*  SageMath
    Openssl
*/

    //ROS_INFO("vector: (%f,%f)",vector.pose.orientation.y,vector.pose.orientation.x);
    publicar(lineal,angular,velocity_pub);

    last_time = current_time;
  }
}