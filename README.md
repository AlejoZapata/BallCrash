# BallCrash
Ball Crash es un juego competitivo para dos jugadores, donde cada uno tiene un control para realizar tres acciones: moverse a la derecha, moverse a la izquierda y disparar. El objetivo es marcar goles en la portería contraria, y el primer jugador en llegar a seis puntos gana. El juego se desarrolla en una pista cerrada, con sensores en cada portería para contar los goles y actuadores en los jugadores para permitir su movimiento. Además, los displays muestran la puntuación y el ganador en tiempo real. Este proyecto busca ofrecer una experiencia divertida y dinámica, combinando control físico e interacción tecnológica.
<Requisitos Funcionales - Ball Crash



Procesamiento de Datos

   
La Raspberry Pi Pico debe procesar la entrada de los controles en tiempo real.

   
Los algoritmos de detección de goles deben identificar cuándo la pelota cruza la línea de gol mediante sensores.

   
Se debe actualizar la puntuación y mostrarla en una pantalla digital.



Comunicación

   
Los controles inalámbricos deben enviar comandos a la Raspberry Pi Pico con baja latencia.

   
La Raspberry Pi Pico debe enviar señales a los actuadores para mover a los jugadores.

   
Comunicación con los displays para mostrar la puntuación y el ganador.



Control

   
Cada jugador debe poder mover su avatar a la izquierda o derecha mediante un control remoto.

   
Los jugadores pueden disparar la pelota con un mecanismo de impulso.

   
El sistema debe detectar automáticamente los goles mediante sensores en las porterías.



Gestión de Energía

   
La Raspberry Pi Pico debe gestionar el consumo energético de los actuadores y sensores.

   
Implementación de modos de ahorro de energía cuando no hay actividad.



Interfaz de Usuario

   
Cada jugador interactúa con un control con tres botones: izquierda, derecha y disparo.

   
Los displays deben mostrar la puntuación en tiempo real.

   
Un indicador debe señalar cuando un jugador ha ganado la partida.



Seguridad

   
El sistema debe evitar fallos en la detección de movimientos para garantizar precisión.

   
Protección ante reinicios inesperados para evitar la pérdida de la puntuación.



Tiempos de Respuesta

   
El tiempo de respuesta entre la acción del jugador y el movimiento del actuador debe ser inferior a 100 ms.

   
La actualización de la puntuación debe reflejarse en menos de 500 ms tras un gol.



Cumplimiento de Estándares

   
Uso de protocolos de comunicación confiables como I2C, SPI o UART.

   
Diseño de hardware seguro para evitar fallos eléctricos.>
<Requisitos No Funcionales - Ball Crash



Rendimiento



   
El sistema debe procesar las entradas de los controles y actualizar el estado del juego en menos de 100 ms.

   
La detección de goles debe realizarse en menos de 500 ms.



Disponibilidad



   
El sistema debe estar operativo durante el 99% del tiempo en sesiones de juego normales.



Fiabilidad



   
El sistema no debe fallar más de una vez cada 50 partidas completas.

   
Debe ser capaz de reiniciarse automáticamente en caso de fallo sin perder la puntuación almacenada.



Mantenibilidad



   
La estructura del código debe permitir futuras mejoras y correcciones sin afectar el funcionamiento actual.

   
El hardware debe ser modular, permitiendo reemplazar componentes sin necesidad de modificar el sistema completo.



Usabilidad



   
Los controles deben ser intuitivos y fáciles de aprender en menos de 5 minutos de uso.

   
La interfaz de los displays debe ser clara y sin elementos distractores.



Consumo de Energía



   
El sistema debe operar con una fuente de alimentación de bajo consumo, no superando los 5W en total.



**Tolerancia a Fallos



   
En caso de interrupción de energía, el sistema debe reiniciarse sin afectar la integridad del hardware o software.



Interoperabilidad



   -El sistema debe ser compatible con diferentes módulos de comunicación estándar para displays y sensores.

>
Escenario de Pruebas - Ball Crash



 
Objetivo de las Pruebas

Validar el correcto funcionamiento del sistema embebido de Ball Crash, verificando los controles, sensores, actuadores, visualización de puntajes y comportamiento general del juego en condiciones normales y extremas.



 
Entorno de Pruebas

Mesa de juego: Superficie plana con las porterías y los mecanismos de los jugadores.

Sensores de portería: Dispositivos que detectan cuando la pelota cruza la línea de gol.

Actuadores de los jugadores: Motores que permiten el movimiento de los jugadores.

Controles cableados: Mandos físicos con tres botones (izquierda, derecha y disparo).

Displays: Pantallas que muestran el puntaje en tiempo real.

Fuente de alimentación: Suministro eléctrico estable con opción de simulación de cortes de energía.


Casos de Prueba



A. Validación del Movimiento de los Jugadores
Prueba 1: Pulsar el botón de movimiento a la izquierda y verificar que el jugador se desplace en esa dirección.

Prueba 2: Pulsar el botón de movimiento a la derecha y verificar que el jugador se mueva en consecuencia.

Prueba 3: Presionar repetidamente los botones de movimiento para evaluar la respuesta del sistema a comandos rápidos.



B. Verificación del Mecanismo de Disparo
Prueba 4: Presionar el botón de disparo y medir la velocidad de respuesta del actuador.

Prueba 5: Evaluar si la fuerza del disparo es suficiente para impulsar la pelota de un extremo de la mesa al otro.

Prueba 6: Simular disparos repetidos para evaluar la durabilidad del sistema mecánico.



C. Detección de Goles
Prueba 7: Introducir la pelota manualmente en la portería para verificar que el sensor la detecte.

Prueba 8: Hacer goles desde distintos ángulos y velocidades para comprobar la precisión del sensor.

Prueba 9: Evaluar si hay falsos positivos (registrar goles sin que la pelota haya entrado).



D. Visualización del Puntaje
Prueba 10: Confirmar que el display aumenta correctamente el puntaje tras cada gol.

Prueba 11: Simular una partida completa hasta que un jugador alcance los 6 puntos y verificar la correcta declaración del ganador.

Prueba 12: Probar un reinicio del sistema y verificar que la puntuación se resetee correctamente.



E. Tiempos de Respuesta
Prueba 13: Medir el tiempo desde que un jugador pulsa un botón hasta que el personaje responde (< 100 ms).

Prueba 14: Medir el tiempo de detección del gol y actualización del marcador (< 500 ms).



F. Robustez del Sistema
Prueba 15: Jugar 10 partidas seguidas sin reiniciar el sistema para evaluar estabilidad.

Prueba 16: Simular un corte de energía y comprobar si el sistema reinicia sin fallos críticos.

Prueba 17: Exponer el sistema a diferentes condiciones de iluminación para evaluar su impacto en la detección de la pelota.
Lista de Componentes y Presupuesto



Componentes Electrónicos



Raspberry Pi Pico - $23,400

Sensores de portería (IR o ultrasónicos) - $19,500 c/u (2 unidades) - $39,000

Motores DC para movimiento de jugadores - $31,200 c/u (2 unidades) - $62,400

Drivers de motor (L298N o similar) - $19,500 c/u (2 unidades) - $39,000

Botones de control (izquierda, derecha, disparo) - $5,850 c/u (6 unidades) - $35,100

Pantalla LCD para puntuación - $46,800

Fuente de alimentación 5V/2A - $31,200

Cables y conectores - $19,500



Componentes Mecánicos



Mesa de juego (acrílico o MDF) - $97,500

Estructura y soporte para sensores - $58,500

Pelota de juego (ligera y resistente) - $7,800



Costos Adicionales



Impresión 3D de piezas (si aplica)** - $78,000

Tornillería y fijaciones** - $19,500

Extras y margen de error** - $39,000



Costo Total Estimado: $596,700 COP



Financiamiento



Los costos se cubrirán mediante:

Aportaciones del equipo de desarrollo.>
