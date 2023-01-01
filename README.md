ROS package that reads NMEA sentences over a specified TCP socket stream.

Currently supports GGA and GST sentences.

Requires pynmea2

Publishes to the /fix topic with /NavSatFix message type
