# Autonomonomous car by C++, I hardly know her

A school project building autonomous vehicles that drive themselves along a
walled track about 1 m wide. The firmware is written in MicroPython for an
ESP32-family board. Each vehicle uses three time-of-flight distance sensors to
stay centered between the walls, a Pololu Motoron motor controller, and a start
module that tells it when to go.

### Code

| Folder | Vehicle | Description |
|---|---|---|
| [snok_1.2](snok_1.2/README.md) | RC car | Ackermann steering with a servo and one drive motor. Detects when it is stuck and reverses out |
| [huggorm_1.0](huggorm_1.0/README.md) | Tracked chassis | Excavator with two tracks, steers by the speed difference between them |

Each folder is a complete firmware. To run one, copy its `config.py`, `main.py`
and `lib/` to the board, for example with Thonny. All tunable values are in `config.py`.

### Components

- VL53L4CX ToF lidar sensor – left and right side sensors
- VL53L0X ToF lidar sensor – front (middle) sensor
- Pololu Motoron M2T550 dual I2C motor controller
- DC-DC step-down converter (1.23–30 V, 1.5 A)
- Slide switch (on-on)
- GT Power B3 battery charger

### PCB-schematics

[Kicad/Bil v 1,0](Kicad/Bil%20v%201,0) contains the KiCad project for the
car's circuit board: the schematic, the PCB layout, and the custom symbol
libraries (`Auto_car_library`, `motron_i2c_3motor`).

### License

See [LICENSE](LICENSE).
