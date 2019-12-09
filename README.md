---
![e-Yantra](logo.png "e-Yantra")
---

# Gas Sensor Interfacing with Firebird Robot

This repository is for interfacing the MQ-2 Gas sensor with the Firebird Robot. You will need both the MQ-2 sensor and the Firebird robot in order to work on this repository. This repository is open to all members of the e-Yantra community. Any member may contribute to this project without being a collaborator.

## How to contribute to this project
Fork this repository and get started. You are supposed to interface the sensor with the Firebird robot. You can use any of the Firebirds GPIO pins. A basic code template is given to help you get started. You are free to edit the code however you like. You must add comments wherever necessary.

Author notes:

Author: Shivam Mahesh Potdar (github/shivampotdar) (shivampotdar99@gmail.com)
This code forms an interface for MQ2 gas sensor with Firebird, values of LPG, CO and Smoke detected in PPM are
shown on the 16x2 LCD onboard.

The MQ-2 gas sensor has three outputs pins, connections can be made as follows :
- The ATMEGA2560 Microcontroller Board expansion socket on Firebird can be used for the purpose:
- Note that all four jumpers from J2 should be removed to make use of the ADC pins on the socket
- This would disable IR Proximity sensor 1-4
- DO pin on the MQ2 module can be left unused as it is used with a comparator on-board just to detect threshold

MQ-2 <---------> Firebird
Vcc             Pin21/22 (5V)
GND				Pin23/24 (Ground)
AO				Pin7-10 (7=ADC6,8=ADC7,9=ADC5,10=ADC4)

References :
1. MQ2 Datasheet - https://www.pololu.com/file/0J309/MQ2.pdf
2. https://www.instructables.com/id/How-to-Detect-Concentration-of-Gas-by-Using-MQ2-Se/
