/**
 * Publicador_brujula.
 * Nodo de ROS que toma los datos de la brujula (hmc58883l) y los publica.
 * Para tomar los datos de la brujula se emplea la librería programada.
 * Desarrollado por PatatitasTeam®
 */


// LIBRERIAS
#include <iostream>                 // Libreria Standar
#include <queue>
#include "ros/ros.h"                // Libreria ROS
#include "std_msgs/Float64.h"  // Libreria Vector3 Mensaje de ROS para X Y Z.
#include "drivers/hmc5883l.cpp"       // Drivers de brujula

// NAMESPACES
using namespace std;

// VARIABLES - CONFIGURACION
const string nodo_name = "nodo_brujula";    // Nombre del nodo.
const string topic_name = "brujula_bruto";  // Nombre del topic donde se va a publicar la brujula.
float tiempo_muestreo = 0.1;                // Ts | Tiempo de muestreo para la lectura de la brujula.
const int mediana_size = 5;
// VARIABLES - CALIBRACION
// Parametros de calibracion de la brujula
double bias[3] = {-112.017978, -39.199350, -97.169059};
// Filas de la matriz
double A_0[3] = {0.089225, -0.001845, 0.006807};
double A_1[3] = {-0.001845, 0.090054, 0.002206};
double A_2[3] = {0.006807, 0.002206, 0.092495};
// VARIABLES - FILTRO MEDIANA
bool publish = false;
deque<float> mediana_queue;
// Valores maximos y minimos para calibrar.
//float max_x=0, max_y=0, min_x=0, min_y=0;
const float max_x=2000, max_y=900, min_x=-3000, min_y=+600;



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


/** 
 * Prototipo de la funcion calibración.
 * Recibe CompassMsg el puntero que es un struct de la clase hmc5883l que tiene datos .x .y .z .angle.
 * Calibra estos valores.
 */
void calibracion(CompassMsg &mag);

/**
 * Prototipo del filtro de la mediana.
 * Se le pasa el float del header de la brujula y devuelve el valor filtrado.
 */
float filtro_mediana(float header);
void maximos_minimos(CompassMsg &mag);
void calibracion_rapida(CompassMsg &mag);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


/**
 * Funcion principal donde se piden los datos a la libreria y se publican una vez tratados.
 */
int main(int argc, char **argv)
{

  // Iniciamos el nodo.
  ros::init(argc, argv, nodo_name);
  ros::NodeHandle nodo;
  ROS_INFO("nodo_brujula creado y registrado");
  // Cargamos el modulo de la brujula.
  HMC5883L brujula;
  CompassMsg brujula_struct;
  int respuesta = brujula.conectamos_brujula();
  ROS_INFO("Modulo de la brujula cargado; (id-i2c = %d)", respuesta);
  // Definimos el publicador.
  ros::Publisher publicador = nodo.advertise<std_msgs::Float64>(topic_name, 0);
  ros::Duration seconds_sleep(tiempo_muestreo);

  // Bucle donde continuamente se va a realizar la lectura y publicacion de datos.
  // Este proceso se realizara cada float tiempo_muestreo
  while (ros::ok())
  {

  	// Tomamos un dato de la brujula a partir de la funcion de la libreria.
    brujula_struct = brujula.get_data();

    // Calibramos el dato obtenido.
	//calibracion(brujula_struct);
	//maximos_minimos(brujula_struct);
	calibracion_rapida(brujula_struct);

    // Aplicamos el filtro de la mediana.
    filtro_mediana(brujula_struct.angle);

    //printf("Magnetometro [x, y, z] = [%d, %d, %d] - Angulo = %f \n", brujula_struct.x, brujula_struct.y, brujula_struct.z, brujula_struct.angle);
    std_msgs::Float64 mensaje_ros;
    mensaje_ros.data = brujula_struct.angle;
    publicador.publish(mensaje_ros);
    ros::spinOnce();
    seconds_sleep.sleep();
  }


  return 0;
}



void calibracion(CompassMsg &mag){
  mag.x -= bias[0];
  mag.y -= bias[1];
  mag.z -= bias[2];
  double new_x = A_0[0]*mag.x + A_0[1]*mag.y + A_0[2]*mag.z;
  double new_y = A_1[0]*mag.x + A_1[1]*mag.y + A_1[2]*mag.z;
  double new_z = A_2[0]*mag.x + A_2[1]*mag.y + A_2[2]*mag.z;
  mag.x = new_x;
  mag.y = new_y;
  mag.z = new_z;
  // TODO modificar el struc.angle para actualizar el header. 
}

void calibracion_rapida(CompassMsg &mag)
{

	// Apply hard iron calibration
	float x1 = mag.x - ( (min_x + max_x) /2 );
	float y1 = mag.y - ( (min_y + max_y) /2 );
	// Apply soft iron calibration
	float x2  = (float)(x1 - min_x) / (max_x - min_x) * 2 - 1;
	float y2  = (float)(y1 - min_y) / (max_y - min_y) * 2 - 1;
	// Calibramos el heading
	//mag.y = -mag.y;
	float angle = atan2((float) y2, (float) x2);
	if(angle < 0)
		angle += 2*PI;
	if(angle > 2*PI)
		angle -= 2*PI;
	angle = angle * 180/M_PI;
	cout << "ANGULO = " << angle << endl;

}


void print_deque(deque<float>& cola)
{
	for (deque<float>::iterator it = cola.begin() ; it != cola.end(); ++it)
		cout << *it << " ";
}


float filtro_mediana(float value)
{

	// Add dato a la cola, si la cola es < tamaño_cola entonces no se borra head.
    mediana_queue.push_back(value);
    if (mediana_queue.size() > mediana_size)
    	mediana_queue.pop_front();
    
    // Imprimimos el vector desordenado
    deque<float> muestras_ordenadas = mediana_queue;
    print_deque(mediana_queue);

    // Ordenamos el vector
    sort(muestras_ordenadas.begin(), muestras_ordenadas.end());

    // Imprimimos el vector desordenado y el resultado de la mediana
    cout << "Vector Ordenado" << endl;
    print_deque(muestras_ordenadas);
    cout << "Resultado de la mediana: " << muestras_ordenadas[mediana_size/2] << endl;

    
    return muestras_ordenadas[mediana_size/2];
}


/*

void maximos_minimos(CompassMsg &mag){

	if (mag.x > max_x)
		max_x = mag.x;

	if (mag.y > max_y)
		max_y = mag.y;

	if (mag.x < min_x)
		min_x = mag.x;

	if (mag.y < min_y)
		min_y = mag.y;

	cout << "Max_x = " << max_x << " // Max_y = " << max_y << " ||||| Min_x = " << min_x << " // Min_y =" << min_y << endl;

}
*/
