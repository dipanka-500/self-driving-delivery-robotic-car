# self-driving-delivery-robotic-car
in this project , we are integrating ardunio,rasapberrypi to mmake a self driving robotics car which will move to the location based on user given location on website and use face comparsion module to detect the person .
Self-Driving Robotics Car
This project implements a self-driving robotics car that navigates to a user-specified location, captures a photo, and performs image comparison.
Features

Autonomous navigation to user-defined coordinates
GPS-based localization
Inertial measurement for improved positioning
Motor control for precise movement
Image capture at the target location
Photo comparison with user-provided image

Hardware Components

Arduino (main controller)
Raspberry Pi (user interface and image processing)
NEO-6M GPS module
MPU6050 (Inertial Measurement Unit)
Optical encoder
Cytron FD04A motor driver
Camera module (connected to Raspberry Pi)

Software Architecture

User Interface (Raspberry Pi):

Accepts user input for target location
Sends location data to Arduino via I2C
Handles image capture and comparison


Navigation System (Arduino):

Processes location data from Raspberry Pi
Integrates sensor data for accurate positioning
Controls motors for navigation



Setup and Installation

Clone this repository
Install required libraries for Arduino and Raspberry Pi
Connect hardware components according to the wiring diagram
Upload Arduino code to the board
Run the Raspberry Pi script

Usage

Power on the robotics car
Connect to the Raspberry Pi interface
Input target location (distance and angle or latitude and longitude)
Upload a reference photo for comparison
Initiate the self-driving sequence
Wait for the car to reach the destination and perform image capture
Review the results of the image comparison

Contributing
Contributions to improve the project are welcome. Please follow these steps:

Fork the repository
Create a new branch
Make your changes and commit them
Push to your fork and submit a pull request


Contact
dipankar deka
