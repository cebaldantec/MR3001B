Códigos utilizados para la resolución del reto "Implementación de Fixture Hidraúlico para
Centro de Maquinado con Mesa de Cinco Ejes" Materia MR3001B

Análisis cinemática de manipuladores 
Robot:  ABB IRB 2600-12/1.85

"DHR" Genera el Denavit-Hartenberg del reto y ayuda a validar el robot con matrices de transformación
"DHPLOTR" genera el diagrama de transformaciones del robot
"RToolbox" Genera el Denavit-Hartenberg usando toolbox, y tambien genera la cinemática directa graficando sin controlador .plot() y con controlador .teach()
"Cinematica_IRB_2600" Genera igualmente el DH de toolbox, pero añade trayectorias y anmación de la cinemática inversa
