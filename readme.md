# Proyecto de la Universidad Carlos III de Madrid.

Realizado por:
- Carlos Barreiro
- Juan Pou
- Francisco Moreno
- Álvaro Saez
- Juan Hernandez

En este proyecto se desarrolla el robot "patatitas", el cual se trata de un robot movil con cuatro motores que puede rotar sobre si mismo. Este robot esta compuesto por una Rasperry PI II, en el cual se instalado el midleware ROS.

## Hardware

- gy521: acelerometro
- hmc5883l: brujula.
- pca9585: pwm (12 canales) mismo reloj.

## Esquema de la carpeta src:

- *carpeta drivers*: esta carpeta contiene ficheros cpp que se emplean como objetos o librerias para controlar el hardware instalado en la Raspberry PI. Podría decirse que sen encuentran los ficheros de más bajo nivel. Más adelante comentaremos más detalladamente el contenido de esta carpeta.

- Los ficheros que comienzan por el prefijo main, se tratan de software de prueba para comprobar individualmente el funcionamiento de las librerias de la carpeta drivers.

- Los ficheros que comienzan por los prefijos ros, publicador o subscriptor son ficheros son ficheros.

- - - 

## Topics:

En este apartado se deberán de añadir los topics del robot patatitas.
