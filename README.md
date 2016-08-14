# 16mm_projector_v2
mod of a 16mm projector for the project "El curso del tiempo" from Xavi Hurtado

This is a modification of a projector (EIKI SSL0L) to take control over the parameters of the projection: tape speed, amount of light and shutter.
For the light we replace the incandescent bulb (300W, lumens: unknown)  by a COB LED (Citizen CLU058-1825C4-303M2K1, 100W about 17000 lumen).
To control the tape speed, We replaced the original motor with a DC brushless servo motor (Trinamic QBL5704-94-04-032 and TMCM-1630 control board).
The shutter originaly was hard linked to the main motor, we eliminated the hard link to have control independant, added a secondary shutter and two DC motors to move them. The link to the main motor and sync was made by a dual PLL system programed on an Arduino M0 board.
The power for these DC motors is delivered trough a Pololu board dual MC33296.
Arduino also controls the communication with the TMCM-1630 trough serial interface, the amount of light by switching the LED with a power
mosfet and receive the control from the computer over an ethernet line in OSC packets (we use here the Arduino Ethernet 2 Wiznet W5500).
The power is supplied by a Mean Well RSP-500-48 raised to 54V, with two DC-DC converters based on the TPS54560 chip from TI.