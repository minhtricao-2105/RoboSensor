clear all variable
clc
close all

a = arduino()

ultrasonicObj = ultrasonic(a, 'D9', 'D10');

while(1)
     buttonState = readDigitalPin(a, 'D7')
end