# What's DCC++ EX
------------

DCC++ EX is an open-source hardware and software system for the operation of DCC-equipped model railroads. It expands on the work of Gregg E. Berman's original DCC++ (which can be found here in the BaseStation-Classic repository)

The system consists of two parts, the DCC++ EX Command Station and one of many front end controllers. These controllers can be hardware controllers (called CABs or Throttles), software applications like JMRI, phone apps like Engine Driver, or our exWebThrottle that is a simple application you run in a browser like a web page and control your model trains.

The DCC++ EX Command Station consists of an Arduino micro controller fitted with an Arduino Motor Shield (or other supported motor controllers) that can be connected directly to the tracks of a model railroad.

# What’s in this Repository
-------------------------

This repository, CommandStation-EX, contains a complete DCC++ EX Commmand Station sketch designed for compiling and uploading into an Arduino Uno, Mega, or Nano.  All sketch files are in the folder named CommandStation-EX and its subforlders. More information about the sketch can be found in the included PDF file.

To utilize this sketch, you can use the following methods: 

* our automated installer
* download a zip file from this repository (green Code button above) and unzip it
* use git clone on this repository

With the manual methods you unzip or git clone to the Arduino IDE
projects folder and then open the file "CommandStation-EX.ino" in the
Arduino IDE. Please do not rename the folder containing the sketch
code, nor add any files in that folder. The Arduino IDE relies on the
structure and name of the folder to properly display and compile the
code. If you do not run the installer, you have to copy
config.example.h to config.h. If you do not have the standard config
you edit config.h according to the help texts in config.h.

The latest production release of the Master branch is 3.0.1:

* Supports the Arduino Uno, Arduino Mega, and Arduino Nano
* Built-in configuration for both the original Arduino Motor Shield, Pololu MC33926 Motor Shield, LMD18200, and BTS7960B
* Built-in configuration and support of Ethernet Shields and the ESP82266 WiFi module (networking for use with Mega only).

For more information on the overall DCC++ EX system, please follow the links in the PDF file.

Detailed diagrams showing pin mappings and required jumpers for the Motor Shields can be found in the Documentation Repository

The Master branch contains all of the Command Station functionality showed in the DCC-EX YouTube channel.

# How to find more information
--------------------------

[DCC++ EX WEB Page](https://dcc-ex.github.io "DCC++ EX WEB Page")

[The DCC++ EX Discord and live support](https://discord.gg/y2sB4Fp "The DCC++ EX Discord Server")

[TrainBoard DCC++ Forum](https://www.trainboard.com/highball/index.php?forums/dcc.177/ "TrainBoard DCC++ Forum")

-May 2020
!!
