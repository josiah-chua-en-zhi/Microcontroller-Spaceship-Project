# Microcontroller-Spaceship-Project
This script was written for a school project to hone our skills in microcontroller programming, by utilizing the different sensors on the on the STM32L475VG board with an ARM Cortex M4 processor along with an external sensor to mimic uses in a spacecraft. Readings from the sensors are printed out to a terminal aka TermiNUS(Tera Term is an option), there are different modes that the microcontroller operates in and interrupts were also configured for the accelerator for free fall, user push button to change modes and disable warnings and IR sensor that acts as a door sensor

It is recommended to run it using the STM32 IDE. The workspace zip folder is in the repo and the project to open is Assignment 2 and the main.c script can be found in core->src.

## Project Requirements

### STATIONARY Mode

- When powered on, the system enters STATIONARY Mode by default. A message "Entering STATIONARY mode" is sent to TermiNUS once. 

- In STATIONARY MODE, only those sensors which monitor passenger cabin, i.e., Termperature, Pressure and Humdidity sensors are read continuously, at a rate of once per second. Accelerometer, gyroscope and compass are not read. The sensor readings (telemetry) are sent to TermiNUS in the format "T:tt.tt, P:pp.pp, H:hh.hh \r\n". The sensors readings to be displayed should not be the raw values given by the sensor directly; it should be the magnitude of the physical quantity. You can choose the unit to use, as long as you understand what the unit means in a physics-sense.

- If any of the readings exceed a certain threshold for that particular sensor - high and/or low thresholds, as you deem appropriate, the system goes into a state of WARNING. The LED will start displaying WARNING_LED, and the system will not poll the sensors anymore until the warning is cleared. The message "STATIONARY mode: WARNING" is sent once every second instead of the telemetry message mentioned above. The system comes out of WARNING only through CLEAR_WARNING, upon which exhibits its usual behavior in the STATIONARY mode.

- CLEAR_WARNING is ignored when the system is not in a state of WARNING.

- If the system is not in a state of WARNING, STATIONARY_LED is displayed.

- CHANGE_MODE causes a count-down sequence to be initiated within 1 second, provided the system is not in a state of WARNING. While the system is counting down, the sensors are still read, and the countdown count is also sent along with telemetry. The format "i, T:tt.tt, P:pp.pp, H:hh.hh \r\n" where 'i' is the countdown count such as 9/8/...0 should be used. If any of the sensor thresholds are exceeded at any instant before the system enters the LAUNCH mode, it goes into the WARNING state and countdown is aborted. The system enters LAUNCH mode 1 second after the system displays 0.

  - 'Within 1 second' means that you can take up to 1 second (i.e., 1 second or less) following CHANGE_MODE to initiate the follow up actions.

### LAUNCH Mode
- A message "Entering LAUNCH mode" is sent once to TermiNUS immediately upon entering the LAUNCH mode.

- In addition to Temperature, Pressure and Humidity sensors, Accelerometer, Gyroscope and Magnetometer measurements are also read at a rate of once a second. The sensor readings (telemetry) are sent to TermiNUS in the format "T:tt.tt, P:pp.pp, H:hh.hh, A:aa.aa, G:gg.gg, M:mm.mm \r\n". For A, G, M sensors, you can choose to send either the most relevant axis (e.g. Z-axis for accelerometer), all the three axes (you can make changes to the recommended format as appropriate), or the overall magnitude [sqrt(x2+y2+z2)].

- A state of WARNING can happen in LAUNCH mode too, similar to that of stationary mode, except that the system can go into a state of WARNING due to any of the six sensors exceeding their thresholds (not just T, P, H sensors). The message "LAUNCH mode: WARNING" is sent once every second instead of the telemetry message mentioned above. The WARNING can be cleared only through CLEAR_WARNING, upon which the system exhibits its usual behavior in the LAUNCH mode.

- If the system is not in a state of WARNING, LAUNCH_LED is displayed.

- CHANGE_MODE causes the system to change to RETURN mode within 1 second, if the system is not in a state of WARNING.

### RETURN Mode
- The system behavior in RETURN mode is very similar to that of LAUNCH mode except that
- The messages will have RETURN instead of LAUNCH.
- CHANGE_MODE causes the system to change to STATIONARY mode within 1 second, if the system is not in a state of WARNING.

### LEDs
- WARNING_LED is LED blinking at a frequency of 2 Hz, i.e., twice per second.
- STATIONARY_LED is LED not blinking at all, i.e., LED is OFF. 
- LAUNCH_LED is LED blinking once every second (frequency = 1Hz), with a duty cycle of 25%.
- RETURN_LED is LED blinking once every second (frequency = 1Hz), with a duty cycle of 75%.

### User Push Button
The rocket/space shuttle is assumed to be controlled by the human passenger through a pushbutton. This has to be read using interrupt. There are two different actions performed using the pushbutton, as explained below
- CHANGE_MODE (double press) - this action corresponds to the pushbutton being pressed twice within a 1 second window, i.e., the time difference between two presses is less than 1 second. If the system is in a state of WARNING, a double press (i.e., there are two presses within a second) is ignored.
- CLEAR_WARNING (single press) - this action corresponds to the pushbutton being pressed only once within a 1 second window when the system is in a state of WARNING. If the system is not in a state of warning, a single press (i.e., there is no second press within 1 second) is ignored.
This means you need to wait for 1 second following a press to ensure that there is no second press, i.e., it is indeed CLEAR_WARNING and not CHANGE_MODE.






