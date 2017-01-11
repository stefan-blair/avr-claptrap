# avr-claptrap

## synopsis
for this project, i designed and built a small, robotic model of the robot "Claptrap" from the videogame "Boarderlands 2".  i constructed the circuits, however the casing and wheels ultimately proved too troublesome to acquire (i considered 3D printing).
the entire device is controlled by an ATMega324p microcontroller, which was programmed in C using the avr library.  

## hardware
* distance sensor, used to prevent the device from bumping into objects and walls
* small speakers, codec chip and small flash storage, used to save various audio clips of "Claptrap" phrases, and play them out loud
* two servo motors, used to control wheels to move around
* ATMega324p, used to monitor all sensors and control all output devices
* ATMega248, used to control two motors based off of input from the ATMega324p

## image
![Alt Text](https://s3-us-west-2.amazonaws.com/stefanblairpersonalsite/avr/File+Jan+11%2C+9+44+07+AM.jpeg)
