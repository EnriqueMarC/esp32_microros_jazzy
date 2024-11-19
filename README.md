# Instalación de Micro-ROS

Después de instalar ROS 2 y configurar correctamente el espacio de trabajo, se puede instalar el agente de micro-ros siguiendo estos pasos:

```
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir uros_ws
cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```


## Creación del agente micro-ROS

```
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Ejecutar el agente de micro-ROS
Para realizar una conexión vía Wi-Fi, usando el protocolo UDP4 se debe ejecutar la siguiente instrucción:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
El puerto de enlace debe ser el mismo configurado en el transport de Arduino. 

Por otra parte, para una ejecución vía serial, se debe ejecutar la siguiente instrucción:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSBx
```
donde `x` corresponde al ID del dispositivo. Para verificar qué ID tiene se puede revisar en Arduino -IDE o también puedes visualizarlo con el comando 
```
ls /dev/ttyUSB*
```
Para ello la ESP32 debe estar conectada a la computadora. Si tienes diferentes dispositivos USB, prueba desconectar el que interesa correr el comando anterior, y verificar cuál desaparece. 

## Creación del workspace del firmware (opcional)
En caso de que se requiera programar la ESP32 directamente en micro-ros, es necesario instalar el ESP-IDF antes de cualquier otra cosa, para ello, sigue los pasos de este [tutorial](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html)

Una vez instalado, procede a crear el firmware para ESP32. Para ello deberás estar en el directorio del espacio de trabajo de micro-ros, es decir `~/uros_ws/` y ejecutar el siguiente comando:
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
```
### Construcción de firmware

```
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

## Añadir librerías de micro-ROS a arduino

Para ello descarga las librerías del siguiente repositorio: https://github.com/micro-ROS/micro_ros_arduino/releases
Si tienes instalada la versión jazzy, no hay inconveniente en instalar la versión para iron (aparece como la v2.0.7-iron). Descarga el archivo que aparece con el nombre __Source code (zip)__.

Posteriormente, abre el IDE de Arduino, ve a __Sketch > Include Library > Add .ZIP Library__ y selecciona la librería que acabas de descargar (micro_ros_arduino-2.0.7-iron.zip).

## Prueba de la librería micro-ros-arduino
Prueba cargando algún ejemplo en la ESP32. Para probar el Wi-Fi, usa el que se llama __micro-ros_publisher_wifi__. 

Dentro del __void setup__ encontrarás lo siguiente:

```
void setup() {
  set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "192.168.1.57", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "topic_name"));

  msg.data = 0;
}
```

Para configurar la conexión de la ESP32 a Wi-Fi, coloca los datos de la red en los espacios que dicen __WIFI SSID__ y __WIFI PASS__. En el primero coloca el nombre de la red y en el segundo la contraseña.

En el tercer campo debes colocar la ip de la computadora que está corriendo el agente de micro-ROS. Para obtener la ip, puedes usar el siguiente comando en una terminal:
```
hostname -I
```
El cuarto elemento sirve para colocar el puerto por el cual se realiza la comunicación. 

Las demáslíneas sirven para configurar otros aspectos del nodo, tales como el nombre del nodo, la creación del publisher, la definición del tipo de dato y el nombre del tópico.

El programa anterior, funciona solamente si el ROS-DOMAIN-ID es 0. Si el ID es diferente entonces la configuración en el __void setup__ quedaría de la siguiente manera

```
void setup() {
  set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "192.168.0.107", 8888);

  // Next two lines could be removed
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();
  
  // Create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  // Set your domain_id
  size_t domain_id = 2;

  RCCHECK(rcl_init_options_init(&init_options, allocator));

  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

  //create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "prueba"));

  msg.data = 0;
}
```

## Installación de docker

Para recompilar las librerías de micro-ROS y las interfacez de ROS2 en el IDE de arduino se debe instalar docker con el siguiente comando:

```
sudo snap install docker
```

## Creación de mensajes personalizados
Como primer paso, se debe crear un nuevo paquete dentro de las librerías de micro-ROS para arduino

```
cd ~/Arduino/libraries/micro_ros_arduino/
pushd extras/library_generation/extra_packages
ros2 pkg create --build-type ament_cmake my_custom_message
cd my_custom_message
mkdir msg
touch msg/MyCustomMessage.msg
```
Dentro del archivo `MyCustomMessage.msg` se debe colocar la definición del mensaje, por ejemplo 

```
float32 elemento1
float32 elemento2
```

En el archivo `CMakelist.txt`, del directorio `/my_custom_message` debe añadirse __antes__ de la instrucción `ament_package()` las siguientes líneas:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMessage.msg"
 )
```
Después, en el archivo `package.xml`, añade las siguientes instrucciones

```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Posteriormente se ejecutan los siguiente comandos:

```
cd ~/Arduino/libraries/micro_ros_arduino/

sudo docker pull microros/micro_ros_static_library_builder:jazzy
sudo docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:jazzy -p esp32
```

Con lo anterior ya deberían aparecer las librerías compiladas en Arduino IDE. Después, será necesario copiar la carpeta que contiene el mensaje personalizado a la dirección `~/uros_ws/src/` y posteriormente compilar el espacio de trabajo. Las siguientes líneas deberían hacer lo anterior:

```
cp -r ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages/my_custom_message/ ~/uros_ws/src/
cd ~/uros_ws
colcon build
```

Para revisar que existen los paquetes en Arduino, puede abrirse el archivo `available_ros2_types.txt`, el cual se encuentra en la ruta `~/Arduino/libraries/micro_ros_arduino/`. 
```
...
lifecycle_msgs/Transition.msg
lifecycle_msgs/TransitionDescription.msg
lifecycle_msgs/TransitionEvent.msg
my_custom_message/Joints.msg
my_custom_message/MyCustomMessage.msg
nav_msgs/GetMap.srv
nav_msgs/GetPlan.srv
nav_msgs/GridCells.msg
nav_msgs/LoadMap.srv
nav_msgs/MapMetaData.msg
...
```
Para hacer lo mismo en ROS 2, se puede ejecutar el siguiente comando
```
 ros2 interface list --only-msgs
```

Esto tiene la siguiente salida:

```
...
map_msgs/msg/PointCloud2Update
map_msgs/msg/ProjectedMap
map_msgs/msg/ProjectedMapInfo
micro_ros_msgs/msg/Entity
micro_ros_msgs/msg/Graph
micro_ros_msgs/msg/Node
my_custom_message/msg/Joints  
my_custom_message/msg/MyCustomMessage
nav_msgs/msg/GridCells
nav_msgs/msg/MapMetaData
nav_msgs/msg/OccupancyGrid
nav_msgs/msg/Odometry
nav_msgs/msg/Path
pcl_msgs/msg/Model
...
```
