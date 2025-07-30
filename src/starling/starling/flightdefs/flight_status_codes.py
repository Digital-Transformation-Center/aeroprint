# flight_status_codes.py
# This file defines named constants for flight state/status codes for use in ROS Int8 messages.

RESERVED = 0

# Mode Status Codes
FLIGHT_DISARMED = 1
FLIGHT_DISARMED_TITLE = "Starling Disarmed"
FLIGHT_DISARMED_DESCRIPTION = "The drone's motors are disarmed and it is safe to approach."
FLIGHT_ENGAGED = 2
FLIGHT_ENGAGED_TITLE = "Scanning Active"
FLIGHT_ENGAGED_DESCRIPTION = "The drone is flying it's computed flight path and scanning an object."
FLIGHT_ARMED = 3
FLIGHT_ARMED_TITLE = "Starling Armed"
FLIGHT_ARMED_DESCRIPTION = "The drone's motors are active. Please stand clear."
FLIGHT_LANDING = 4
FLIGHT_LANDING_TITLE = "Starling Landing"
FLIGHT_LANDING_DESCRIPTION = "The drone is attempting to land below its current position."

# Structure status codes
FLIGHT_PATH_LOADED = 5
FLIGHT_PATH_LOADED_TITLE = "Flight Path Loaded"
FLIGHT_PATH_LOADED_DESCRIPTION = "The drone has successfully computed a flight path based on received parameters."
FLIGHT_PATH_NOT_LOADED = 6
FLIGHT_PATH_NOT_LOADED_TITLE = "Flight Path Failed to Load"
FLIGHT_PATH_NOT_LOADED_DESCRIPTION = "An error occurred while computing a flight path based on received parameters."


FLIGHT_ERROR = -1
FLIGHT_ERROR_TITLE = "Critical Flight Error"
FLIGHT_ERROR_DESCRIPTION = "A critical error has occurred within the flight control node."

STATUS_TITLES = [
    "",
    FLIGHT_DISARMED_TITLE,
    FLIGHT_ENGAGED_TITLE,
    FLIGHT_ARMED_TITLE,
    FLIGHT_LANDING_TITLE,
    FLIGHT_PATH_LOADED_TITLE,
    FLIGHT_PATH_NOT_LOADED_TITLE,
    FLIGHT_ERROR_TITLE
]

STATUS_DESCRIPTIONS = [
    "",
    FLIGHT_DISARMED_DESCRIPTION,
    FLIGHT_ENGAGED_DESCRIPTION,
    FLIGHT_ARMED_DESCRIPTION,
    FLIGHT_LANDING_DESCRIPTION,
    FLIGHT_PATH_LOADED_DESCRIPTION,
    FLIGHT_PATH_NOT_LOADED_DESCRIPTION,
    FLIGHT_ERROR_DESCRIPTION
]

