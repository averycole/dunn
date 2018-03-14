# US Digital Encoders
This is a ROS package used to read and publish data from the US Digital Encoders. The published messages depend on the configuration of the encoders on the vehicle. Currently, there are nodes for the Taylor Dunn vehicle (one steering, one drive encoder) and the UGPS trailer (left and right drive encoders). There is also a speedometer node which can transform the output of the other nodes into a speedometer (complete with simple GUI)!

## Calibration
When using the speedometer, make sure the correct encoder is being polled and the correct number of ticks per metre is used to calculate the speed. See *speedometer.py* for details.