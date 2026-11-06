Migration Plan
Do not rewrite the entire application in one pass. Use this order:
1. Install and pin an ESP-IDF version.
2. Create an empty native ESP-IDF project.
3. Make app_main() build, flash, and print a log message.
4. Add GPIO and LED control.
5. Add I2C and validate the MPU6050 independently.
6. Add SPI and validate the BMP280 independently.
7. Add LEDC and validate each servo with the motor/fin mechanism disconnected.
8. Add UART and validate GPS parsing.
9. Port the navigation math without hardware dependencies.
10. Add sensor-to-navigation queues.
11. Add the flight-control task.
12. Add actuator arming and safety interlocks.
13. Add telemetry and fault handling.
14. Only then test the complete system.