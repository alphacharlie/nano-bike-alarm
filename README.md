# nano-bike-alarm
Arduino based Bike/eBike/motorcycle anti-theft alarm using theArduino Nano V3, MPU6050, Adafruit FONA808, 
and the MRFID522 Mifare RFID reader. 

## Hardware 
Thefollowing hardware is used in this project-
 - Arduino Nano V3
 - MPU6050 accelerometer breakout
 - MFRC-522 RFID reader
 - Mifare 1K classic PICC RFID cards
 - Adafruit FONA-808 GSM/GPRS/GPS mdoule
 - 3.7V lipo flat pack battery with JST-PH connector
 - 3V->5V Boost power supply (about $5 on ebay)

## Wiring
Each component used in this sketch has a different communication protocol. The FONA uses serial, the MPU uses I2C and the MFRC uses SPI. The only points on the Arduino that have multiple connections are 5V and Gnd. So it is possible to wire point-to-point without pin-headers and end up with a pretty small package. (Small enough to hide inside a bicycle seat anyway...)

See WiringDiagram.png for wiring details.

## Prerequisites
This sketch uses the following Arduino Libraries and will not compile without them-
 - I2CDevLib: https://github.com/jrowberg/i2cdevlib
 - AdaFruit FONA Library: https://github.com/adafruit/Adafruit_FONA_Library
 - RunningAverage Library(included in libraries dir): http://playground.arduino.cc/Main/RunningAverage

## Software
This project consists of 2 sketches- Bike-Alarm and newKey. Bike-Alarm is (as you might imagine) the alarm sketch. 'newKey' provisions a new PICC card.

## Install
To install, first assemble and test the hardware. Once you have the hardware complete, extract the package and open the 'newKey' sketch.

In the Arduino IDE, edit this sketch and change the hex values for the newA and newB keys and for the tokenBlockData array. The keys are 6 bytes long. THe token block is 16 bytes. To generate random bytes for your key and tokn you can use 
https://www.random.org/bytes/. Once you have your keys and token you can compile and run the 'newKey' sketch. Once the sketch is running, connect to the Serial monitor in the Arduino IDE and follow the prompts to provision your PICC. At the end of the process if it's complete you will be given the UID of the PICC card. Write that down for the next step.

Once that is complete open the Bike-Alarm sketch and scroll to the USER CONFIG section and-
 - Set your keys to the key byte array(s) you generated for the newKey sketch
 - set your tokenBlockData to the values you used for the newKey sketch
 - place your key UID in one row of the 'keyUIDs' array (make sure the number of rows amtches the N_UIDS var or the sketch won't compile.
 - put your cell # in the 'alertphone' variable

Once it's set up, the last thing to do is install it on your bike. It can be hidden in the seat or for eBikes inside the battery box. (If your battery box is metal, you may need to futz with antenna and RFID reader placement to ensure you get a GPRS signal and GPS fix.) 

You'll want some sort of pocket to hold the PICC card next to the reader under your seat or on the battery box.

##Usage
To use you simply place the provisioned PICC (card) in it's 'pocket' next to the reader. Once the reader authenticates the PICC, the alarm is disarmed. Removing the PICC arms the alarm.

When armed, the Arduino turns on the GPS, then scans the output from the MPU6050. If motion is sensed an SMS with the GPS coordinated, heading and speed of the bicycle are sent to the alert phone. SMS are sent every 3 minutes until motion stops or the alarm is disarmed. (I may change this to FORCE periodic SMS after alarm is triggered and not just if motion continues. I haven't decided yet.)

## Troubleshooting
<<<<<<< HEAD
IF you run into problems there is disabled debugging code in the sketch that can be used with the serial monitor to figure things out. But enabling the DEBUG #ifdefs is unlikely to work because the sketch uses too much space. So you'll probably need to enable the messages you need 'a la carte'. 
=======
IF you run into problems there is disabled debugging code in the sketch that can be used with the serial monitor to figure things out. But enabling the DEBUG #ifdefs is unlikely to work because the sketch uses too much space. So you'll probably need to enable the messages you need 'a la carte'. 


>>>>>>> origin/master
