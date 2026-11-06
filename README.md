# Robot diferencial en Gazebo — ET06

**Integrantes:** Antoine Meunier y Diego Oxman  
**Asignatura:** Robótica — Universidad EIA

## Descripción

Simulación de un robot diferencial con dos ruedas motrices, un apoyo
esférico pasivo, un LiDAR y una cámara.

El robot se controla con el teclado y recorre una pista con un
estrechamiento, un obstáculo y dos giros.

Un nodo propio calcula la odometría a partir de las velocidades de las
ruedas y la compara con la odometría del plugin DiffDrive.

## Entorno

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3

Paquetes necesarios:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard
```

## Estructura

```text
src/mi_robot_diferencial/
├── config/
│   ├── bridge.yaml
│   └── control.rviz
├── launch/
│   └── mi_robot.launch.py
├── scripts/
│   └── comparar_odom.py
├── urdf/
│   └── mi_robot.urdf.xacro
├── worlds/
│   └── pista.sdf
├── CMakeLists.txt
└── package.xml
```

## Compilación y ejecución

Abrir una terminal en la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=26
export GZ_PARTITION=antoine_diff

colcon build --symlink-install
source install/setup.bash

ros2 launch mi_robot_diferencial mi_robot.launch.py
```

Cuando se abra Gazebo, pulsar **Play** para iniciar la simulación.

El launch abre Gazebo y RViz, crea el robot e inicia el bridge,
robot_state_publisher y el nodo de comparación de odometría.

## Teleoperación

En otra terminal, desde la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=26
export GZ_PARTITION=antoine_diff

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Mantener seleccionada la terminal del teclado.

- `i`: avanzar.
- `,`: retroceder.
- `j` y `l`: girar.
- `k`: detenerse.
- `z`: reducir la velocidad.

Seguir las instrucciones que aparecen en la terminal.

## Topics principales

| Topic | Función |
|---|---|
| `/cmd_vel` | Comando de velocidad |
| `/joint_states` | Posiciones y velocidades de las ruedas |
| `/odom` | Odometría de DiffDrive |
| `/odom_kinematic` | Odometría calculada por nuestro nodo |
| `/scan` | Mediciones del LiDAR |
| `/image_raw` | Imagen de la cámara |
| `/camera_info` | Parámetros de la cámara |
| `/tf` y `/tf_static` | Transformaciones entre los frames |
| `/clock` | Tiempo de simulación |

El nodo `comparar_odom` muestra periódicamente el error de posición
y el error angular en la terminal del launch.

## Pruebas realizadas

- Avance y retroceso.
- Rotación sobre el eje del robot.
- Visualización del robot, LiDAR y cámara en RViz.
- Comparación de las dos odometrías.
- Recorrido de la pista mediante teleoperación.

En uno de los ensayos, el mayor error entre los valores registrados fue
de 0,0127 m en posición y 0,0328 rad en orientación.

Estos resultados comparan dos estimaciones de odometría, no la pose real
del robot en Gazebo. Los mensajes comparados no están sincronizados
exactamente.

## Simplificaciones

- El apoyo pasivo es una esfera fija con bajo rozamiento, no una rueda
  caster con mecanismo de giro completo.
- Los sensores son modelos simplificados basados en el tutorial.
- La integración de la odometría utiliza Euler.
- No se implementan SLAM ni navegación autónoma.

## Documentación y fuentes

El proyecto parte de la guía del profesor:
“Guía paso a paso Robot diferencial en Gazebo con LiDAR y cámara”,
disponible en el Notion de la asignatura, semana 12.

Se utilizó asistencia de IA para comprender errores, revisar código
y organizar la documentación. Los cambios se probaron en simulación.

