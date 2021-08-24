# meteostation-lcd

This project is intended to automate climate control in the greenhouse for growing mushrooms.
## Sensors
- It uses few sensors for measurement:
1. BME280 sensor for measure humidity
2. 2 DS18B20 sensors for measure compost and air temperature
3. CO2 sensor - МН-Z19B
## Relays
System contains three relays that use PID for given setpoint support:
1. Air temperature relay
2. Humidity relay
3. CO2 relay (inversed, because it controls ventilation that decreases CO2)
## Protocol
It was developed simple protocol, that allows to set and save setpoint values, coefficient (for PID relay controller). All this data is stored in non-volatile memory.
Also it allows to control MHZ19B sensor (it allows to calibrate it and disable autocalibration)
- Sensors data is gathered by arduino (I used arduino nano), displayed on LCD 1602 display, and sent via Serial port
- Data sent by arduino is gathered by script on raspberry pi server and it is written to postgres database. After that it can be used 
for display, analyze, alerts and etc.
## Results
- The results should be something like that:\
<img src="assets/mushrooms_1.jpg?raw=true" height="150" alt="Mushrooms">
<img src="assets/mushrooms_2.jpg?raw=true" height="150" alt="Mushrooms">
<img src="assets/mushrooms_3.jpg?raw=true" width="150" alt="Mushrooms">
- Dashboard with data from Grafana for two days:\
<img src="assets/grafana_1.jpg?raw=true" alt="Time series">
You can see the data plot on the top of the dashboard, and the specific oscillations that are caused by the PID controller. Also, there are last values from sensors displayed on the left side of the dashboard, and CO2 data plot with alert levels on the right bottom side of the dashboard.
- Dashboard with day average compost and air temperatures:
<img src="assets/grafana_2.jpg?raw=true" alt="Time series">