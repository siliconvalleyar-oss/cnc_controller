# Controlador CNC para Raspberry Pi (bcm2835)

Este proyecto implementa un controlador CNC completo para Raspberry Pi (probado en Raspberry Pi 2W) utilizando la librería `bcm2835`. Permite controlar motores paso a paso (hasta 4 ejes: X, Y, Z, A) mediante señales STEP/DIR, gestionar finales de carrera, controlar la velocidad del husillo mediante PWM, y ejecutar archivos G-code.

## Características

- Control de 4 ejes (X, Y, Z, A) con aceleración/desaceleración básica.
- Manejo de finales de carrera (endstops) con parada de emergencia automática.
- Control PWM del husillo (hasta 10.000 RPM configurable).
- Interpretación de comandos G-code (G0, G1, G4, G28, G90, G91, M3, M5, M30).
- Modo de control manual mediante teclado (WASD + Enter/ESC).
- Hilo independiente para monitoreo de finales de carrera.
- Parada de emergencia y pausa/reanudación.

## Hardware requerido

- Raspberry Pi 2W (cualquier modelo compatible con bcm2835 debería funcionar).
- Drivers de motores paso a paso (ej. Pololu A4988, DRV8825) para cada eje.
- Fuente de alimentación adecuada para motores y lógica.
- Finales de carrera (interruptores normalmente cerrados o abiertos, se configuran como pull-up internos).
- Husillo con control PWM (ej. husillo de router de 12-48V con driver que acepte señal PWM).
- (Opcional) Pantalla OLED I2C y botones USB para interfaz.

### Conexiones por defecto (BCM GPIO)

| Eje | Step GPIO | Dir GPIO | Endstop GPIO |
|-----|-----------|----------|--------------|
| X   | 17 (P1-11) | 27 (P1-13) | 10 (P1-19)  |
| Y   | 22 (P1-15) | 23 (P1-16) | 9  (P1-21)  |
| Z   | 24 (P1-18) | 25 (P1-22) | 11 (P1-23)  |
| A   | 5  (P1-29) | 6  (P1-31) | 8  (P1-24)  |

| Husillo PWM | GPIO12 (P1-32) |
|-------------|----------------|

**Nota:** Los pines de endstop se configuran con pull-up interno (activo a nivel bajo cuando se dispara).

## Software requerido

- Raspberry Pi OS (32 bits recomendado).
- Librería bcm2835 (http://www.airspayce.com/mikem/bcm2835/).
- Compilador g++ con soporte C++11 o superior.
- Make (opcional, se puede compilar manualmente).

### Instalación de bcm2835

```bash
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.73.tar.gz
tar zxvf bcm2835-1.73.tar.gz
cd bcm2835-1.73
./configure
make
sudo make check
sudo make install
