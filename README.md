# PoolControl
Project to control the water & heat pump according to the size of the pool.
All this can be configured over a html interface.
A distance control can be done over telegram

## Implemented (TODOS)
- [x] NTP TIME CONTROL
- [x] Webside for Value change and control
- [x] Light Control ON/OFF
- [x] Control Schwitches and Relais
- [x] Light Control ON/OFF
- [x] Telegram for Information and action
- [x] OTA WebUpdater
- [x] Quality of reception of WiFi Signal
- [ ] Pressure Control for Filter
- [ ] PH Measurement
- [ ] Redox Measurement
- [ ] PH & Redox analysis and graph
- [ ] Level detector in the skimmer
- [ ] RF Control for the opening and closing of the terrasse
- [ ] Implement EMONCS for Monitoring
- [ ] Power Management (Measure Current on Pump & PAC Phase and monitor over EMONCS)


## External Sensor BME280
The external sensor is build in a self made housing made of pot coaster
![External Sensor](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4608.jpg)


## HW
During the development I used this kind of bread board
![Development Board](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4103.jpg)

After a functional check I mounted it into the original Housing.
![Final Board1](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4117.jpg)
![Final Board2](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4118.jpg)
I discover that the electrical noise on the switching relais and Interrupt was high and therefore I implemented some inductors to reduce it. An update on the Interrupt code was also mandatory to have a stable functionality. The error was that by switching on and off the MCU got's triggered and did some crayz things and chrashed from time to time.

### The Relais Holder was made with
![Relais1](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4106.jpg)
![Relais2](https://github.com/schnatte/PoolControl/blob/master/Pictures/IMG_4105.jpg)
