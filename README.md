# Mobile-Robots-2026-1
Software para el curso "Robots Móviles" de la Facultad de Ingeniería, UNAM, 2026-1

## Requerimientos

* Ubuntu 24.04: https://ubuntu.com/download/desktop/thank-you?version=24.04.3&architecture=amd64&lts=true
* ROS Jazzy Jalisco: https://docs.ros.org/en/jazzy/Installation.html

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2026-1
* $ cd Mobile-Robots-2026-1
* $ ./Setup.sh
* $ cd ros2_ws
* $ colcon build && source install/local_setup.bash
* $ echo "alias cb='colcon build && source install/local_setup.bash'" >> ~/.bashrc
* $ source ~/.bashrc

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ cd Mobile-Robots-2026-1/ros2_ws
* $ cb
* $ ros2 launch gazebo_envs house_simul.launch.py

Si todo se instaló y compiló correctamente, se debería ver un visualizador como el siguiente:
![rviz](https://github.com/mnegretev/Mobile-Robots-2026-1/blob/main/Media/rviz2.png)

Un ambiente simulado como el siguiente:
![gazebo](https://github.com/mnegretev/Mobile-Robots-2026-1/blob/main/Media/gz.png)

Y una GUI como la siguiente:
![GUIExample](https://github.com/mnegretev/Mobile-Robots-2026-1/blob/main/Media/gui.png)
