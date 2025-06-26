# flight_status_codes.py
# This file defines named constants for flight state/status codes for use in ROS Int8 messages.

RESERVED = 0

# Mode Status Codes
FLIGHT_DISARMED = 1
FLIGHT_ENGAGED = 2
FLIGHT_ARMED = 3
FLIGHT_LANDING = 4

# Structure status codes
FLIGHT_PATH_LOADED = 5
FLIGHT_PATH_NOT_LOADED = 6


FLIGHT_ERROR = -1

# Add more as needed for your application
