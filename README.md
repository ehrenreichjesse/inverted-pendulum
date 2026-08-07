# Inverted Pendulum (Furuta) — LQR Control

Single inverted pendulum built on an ESP32, using an PID and LQR controller, as well as a simple 
swing up algorithm. PID and LQR derived and tuned in Python before deployment to hardware.

## Hardware
- ESP32
- 24V brushed DC motor + BTS7960 H-bridge driver
- Quadrature incremental rotary encoder

## Repo Structure
- `firmware/` — PlatformIO projects (ESP32 firmware for PID and LQR)
- `simulation/` — Python PID and LQR simulation
- `cad/` — CAD files of pendulum
