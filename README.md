# Guide-wire-tether-robot
A car that follows a wire based on the magnetic field produced by the wire

Introduction: design, build, program, and test a
controlled autonomous robot using the magnetic field generated by a guide wire. 


1. Two different microcontroller Systems: For this project you’ll need at least two
microcontroller systems: one microcontroller is used in the robot while the other
microcontroller is used in the guide wire magnetic field generator/transmitter. The
microcontrollers used for the robot controller and guide wire controller must be from
different families. 

2. Battery operated: The robot and its controller must be battery operated.

3. Robot construction: You can use any material you find available for the chassis of the
robot (paper, cardboard, wood, plastic, metal, etc.).

4. C programming: The code for this project must be completed using the C programming
language. 

5. MOSFET drivers: To drive and control the motors of your robot you must use
MOSFETs (Metal Oxide Semiconductor Field Effect Transistors)

6. Guide Wire Generator/Transmitter: While one microcontroller system is used to
control the robot, another microcontroller system is used to generate/transmit a signal at a
constant frequency into the guide wire. The robot controller reads this signal using a
couple of inductive sensors and determines its position from the magnetic field generated
by the guide wire adjusting its position accordingly. Additionally, the guide wire
controller/transmitter must be able to send commands to the robot via the generated
magnetic field. Your design must support at least these six commands:
1) Stop. This command instructs the robot to stop moving.
2) Next intersection, turn right. This command instructs the robot to turn right
after it detects the next intersection.
3) Next intersection, turn left. This command instructs the robot to turn left after
it detects the next intersection.
4) Move Forward. This command instructs the robot to start moving forward.
5) Move Backward. This command instructs the robot to start moving backward.
6) Rotate 180o
.