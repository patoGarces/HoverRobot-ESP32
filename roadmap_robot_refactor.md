# Roadmap de Refactorización - Proyecto Robot con App Android y ESP32

## Etapa 1: Transición a Router Externo
**Prioridad alta** - Cambiar de la configuración actual de ESP32 como AP a un sistema donde el ESP32 y la app Android estén conectados a un **router externo**.

### 1. Configuración del Router Externo:
- Configura el **router externo** con **IP fija** para la app Android, el ESP32 y la Raspberry Pi (si es necesario).
- Asegúrate de que cada dispositivo (app Android, ESP32, Raspberry Pi) pueda obtener una **IP estática** desde el router para evitar problemas con direcciones IP cambiantes.

### 2. Actualización de la App Android:
- Refactoriza la app Android para que, ahora que está conectada al router externo, pueda actuar como **cliente TCP/IP**.
- Elimina cualquier código relacionado con la conexión directa a un **ESP32 AP**.
- Configura las IPs de la app y el ESP32 para comunicarse a través del router.

### 3. Actualización del ESP32:
- Configura el **ESP32 como cliente WiFi** en lugar de Access Point (AP).
- Conéctalo al **router externo** y asegúrate de que pueda obtener una IP estática desde el router.
- Asegúrate de que el ESP32 pueda **enviar y recibir datos** a través del router usando TCP/IP y puertos correctos.

---

## Etapa 2: Ajuste en la Comunicación (TCP/IP)
**Prioridad media** - Ajustar los puertos y la lógica de comunicación entre la app Android y el ESP32, asegurándose de que la comunicación no se vea afectada por el cambio de red.

### 1. Refactorización de la Comunicación TCP/IP:
- Asigna **puertos específicos** para la comunicación entre la app y el ESP32, asegurando que los datos en tiempo real y los comandos sean enviados y recibidos correctamente.
- Ajusta las conexiones TCP/IP para usar los puertos configurados en la app y el ESP32.

### 2. Asegurar la Comunicación Fluida:
- Prueba el sistema para asegurar que los datos en tiempo real del robot sigan fluyendo correctamente entre la app y el ESP32 a través del router.
- Realiza pruebas de latencia y estabilidad en la red.

---

## Etapa 3: Integración de la Raspberry Pi (Navegación y Point Cloud)
**Prioridad baja, pero esencial** - Esta etapa se centra en integrar la Raspberry Pi para la generación de la nube de puntos y navegación.

### 1. Configurar la Raspberry Pi:
- Configura la Raspberry Pi para que pueda conectarse a la red a través del **router externo**.
- Asegúrate de que la Raspberry Pi obtenga una **IP fija** desde el router.

### 2. Establecer Comunicación entre ESP32 y Raspberry Pi:
- Configura el **ESP32 como servidor** para la Raspberry Pi (en el puerto 5678 o el que determines) y que la Raspberry Pi actúe como cliente TCP/IP.
- Desarrolla la lógica para **enviar y recibir datos** entre la Raspberry Pi y el ESP32 relacionados con la navegación y los datos de la nube de puntos.

### 3. Generación y Envío de Point Cloud desde Raspberry Pi:
- Desarrolla o ajusta el código en la Raspberry Pi para generar la **nube de puntos** usando la cámara estéreo y enviar los datos al **ESP32**.
- Asegúrate de que la Raspberry Pi pueda **enviar datos de la nube de puntos** de forma eficiente al ESP32 para que puedan ser procesados correctamente.

---

## Etapa 4: Optimización y Pruebas de Funcionalidad Completa
**Prioridad alta** - Asegurar que todo funcione de manera fluida y optimizada antes de continuar con otros aspectos.

### 1. Pruebas de Integración Completa:
- Con todo el sistema configurado, realiza pruebas exhaustivas para asegurarte de que:
  - La app Android recibe los datos en tiempo real desde el ESP32.
  - La app Android recibe y visualiza correctamente la nube de puntos desde la Raspberry Pi.
  - No haya interferencia entre las comunicaciones (por ejemplo, entre datos del robot y datos de la navegación).

### 2. Optimización de la Comunicación y Red:
- Si es necesario, realiza optimizaciones en la forma de enviar los datos para evitar problemas de latencia o pérdida de paquetes.
- Asegúrate de que el tráfico de datos no sature la red ni cause cuellos de botella.

---

## Etapa 5: Documentación y Backup
**Prioridad baja** - Aunque puede hacerse en paralelo, esta etapa es clave para asegurar que puedas recuperar y mantener tu proyecto en el futuro.

### 1. Documentar Configuraciones y Red:
- Documenta cómo está configurada la red, cómo se conectan la app, el ESP32 y la Raspberry Pi, y qué puertos están siendo utilizados.

### 2. Crear un Backup de Configuraciones:
- Realiza un **backup completo** de las configuraciones de la red y los dispositivos (ESP32, Raspberry Pi, app Android).
- Considera crear una **imagen del sistema** de la Raspberry Pi para facilitar su reinstalación en otro dispositivo.
