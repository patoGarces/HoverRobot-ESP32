# HoverRobot MainBoard Firmware

Firmware para el control del balancing robot utilizando ESP32 y el SDK de Espressif ESP-IDF.

Parte del proyecto [HoverRobot ESP32](https://github.com/patoGarces/HoverRobot_ProyectoFinal).

## Descripción

Este repositorio contiene el firmware necesario para controlar el balancing robot, un tipo de robot que utiliza algoritmos de control para mantenerse equilibrado en posición vertical. El firmware está desarrollado en C con FreeRTOS y el SDK de espressif, utizando el entorno PlatformIO.

## Características Principales

- **Comunicación con la app android**: Comunicación bidireccional con la [aplicación Android](https://github.com/patoGarces/HoverRobotApp-balancing-robot/) a través de socket TCP IP.

- **Control PID**: Implementación de un controlador PID (Proporcional-Integral-Derivativo) para mantener el robot equilibrado.  

- **Ajuste parametros PID en tiempo real**: Posibilidad de ajustar parámetros como los coeficientes del controlador PID y los límites de seguridad.

## Hardware Compatible

Este firmware es compatible con dos configuraciones de hardware:

- **Prototipo con Motores Paso a Paso**: Utiliza motores paso a paso.
  
- **Placa Controladora de Hoverboard**: Utiliza motores brushless de hoverboard con su placa controladora. Puedes encontrar el firmware modificado para esta placa en el repositorio [Firmware Mainboard](https://github.com/patoGarces/hoverrobot-firmware-hack-FOC).

Para seleccionar que hardware se utilizará, se debe comentar/descomentar las lineas 9 y 10 en include/main.h

## Instalación y Uso

Este repositorio hace uso de 3 submodulos principales:

- **TCP_SOCKETS_ESPIDF**: Componente para la comunicación TCP con el ESP32.
  - Repositorio: [TCP_SOCKETS_ESPIDF](https://github.com/patoGarces/TCP_SOCKETS_ESPIDF)
    
- **MCB-component-ESP32**: Componente para la comunicacion con la controladora del hoverboard y el ESP32.
  - Repositorio: [MCB-component-ESP32](https://github.com/patoGarces/MCB-component-ESP32)

- **WS2812**: Componente para controlar los led de estado.
  - Repositorio: [WS2812](https://github.com/patoGarces/WS2812).

Y se debe incluir como componente local: 
- **i2cdevlib**: Librería para dispositivos I2C utilizada en el proyecto.
  - Repositorio: [i2cdevlib](https://github.com/jrowberg/i2cdevlib)

### Requisitos Previos

- Plataforma ESP-IDF instalada en tu ordenador.
- Visual studio code con la extensión de PlatformIo y espidf
- Cualquier ESP32 compatible con bluetooth standard(no ESP32s3).
- Cable USB para la conexión entre tu PC y el ESP32.

### Pasos para la Instalación

1. Clona este repositorio en tu máquina local.
2. Ejecuta los siguientes comandos para agregar los submódulos dentro de `/components`:

    ```bash
    git submodule add https://github.com/patoGarces/TCP_SOCKETS_ESPIDF components/TCP_SOCKETS
    ```

    ```bash
    git submodule add https://github.com/patoGarces/MCB-component-ESP32 components/MCB-component-ESP32
    ```

    ```bash
    git submodule add https://github.com/patoGarces/WS2812 components/WS2812
    ```

3. Abre el proyecto con Platformio.
4. Compila y carga el firmware en tu ESP32.


## Contribuciones

¡Toda colaboración es bienvenida! Si tienes ideas para mejorar este firmware, has encontrado algún error o simplemente quieres contribuir al proyecto, aquí hay algunas formas en las que puedes hacerlo:

- **Envía un Pull Request**: Si tienes cambios que te gustaría agregar al proyecto, no dudes en enviar un pull request. ¡Estaremos encantados de revisarlo y fusionarlo si es apropiado!

- **Hacer un Fork**: Si prefieres trabajar en tu propio espacio de desarrollo, puedes hacer un fork de este repositorio y trabajar en tus propias modificaciones. ¡No dudes en compartir tu trabajo con la comunidad!

- **Abrir un Issue**: Si encuentras algún problema o tienes alguna idea para mejorar el firmware, no dudes en abrir un issue en el repositorio. Estaremos encantados de escuchar tus comentarios y ayudarte a resolver cualquier problema que encuentres.

## Agradecimientos

Gracias a [jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib) por su excelente librería para utilizar el DMP del MPU6050.

## Contacto

Para cualquier consulta o sugerencia, no dudes en contactarnos a través de [correo electrónico](mailto:patricio.garces@outlook.com) o en nuestra página de [GitHub](https://github.com/patoGarces).

## Licencia

Este proyecto está bajo la Licencia Pública General de GNU versión 3 (GPLv3). Esto significa que cualquier persona que modifique o distribuya este software debe hacerlo bajo los términos de la GPLv3, lo que garantiza que las versiones modificadas también sean de código abierto.

Puedes ver los detalles completos de la licencia en el archivo [LICENSE](LICENSE).
