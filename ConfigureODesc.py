import odrive
import time

# Todo list:
# - Set under and over voltage limits ✔️
# - Set current limit ✔️
# - Configure motor ✔️
# - Configure encoder ✔️
# - Configure controller ✔️
# - Configure CAN ✔️

print("Searching for drive...")

drv = odrive.find_any()
print("Drive found!")


# Voltage limits
drv.config.dc_bus_undervoltage_trip_level = 3.3 * 6 # Volts
drv.config.dc_bus_overvoltage_trip_level = 4.25 * 6 # Volts

# Current limits
drv.config.dc_max_positive_current = 20 # Amps
drv.config.dc_max_negative_current = -0.5 # Amps

# Motor config
drv.axis0.motor.config.motor_type = 0 # PMSM_CURRENT_CONTROL
drv.axis0.motor.config.pole_pairs = 7
drv.axis0.motor.config.torque_constant = 8.27 / 360
drv.axis0.motor.config.calibration_current = 7 # Amps
drv.axis0.motor.config.current_lim = 7 # Amps (too high?)

# # Calibrate motor
# input("Going to calibrate motor, press any key to continue: ")

# drv.axis0.requested_state = 4 # AxisState.MOTOR_CALIBRATION

# # Wait for calibration to finish
# while drv.axis0.requested_state == 4:
#     pass

# drv.axis0.motor.config.pre_calibrated = True

# Next actions depend on type of motor
controllerType = input("Drive (d) or Azimuth (a)? ")

if controllerType == 'd':
    drv.axis0.encoder.config.mode = 0 # EncoderMode.INCREMENTAL
    drv.axis0.encoder.config.cpr = 4000 # Counts per revolution
    # drv.axis0.encoder.config.use_index = True

    # drv.config.startup_encoder_index_search = True # Need to find index on startup

    # Setup for azimuth encoder
    drv.axis1.encoder.config.mode = 257 # EncoderMode.SPI_ABS_AMS
    drv.axis1.encoder.config.cpr = 2**14 # cpr
    drv.axis1.encoder.config.abs_spi_cs_gpio_pin = 6 # Use pin 6 for the chip select

    drv.axis0.controller.config.control_mode = 2 # ControlMode.VELOCITY_CONTROL
    drv.axis0.controller.config.vel_limit = 50

    # Dont need position gain cause these just run in velocity control mode
    drv.axis0.controller.config.vel_gain = 0.047 # Tuned

elif controllerType == 'a':
    drv.axis0.encoder.config.mode = 257
    drv.axis0.encoder.config.cpr = 2**14
    drv.axis0.encoder.config.abs_spi_cs_gpio_pin = 6

    drv.axis0.controller.config.control_mode = 3 # ControlMode.POSITION_CONTROL
    drv.axis0.controller.config.vel_limit = 50

    drv.axis0.controller.config.pos_gain = 60 # Tuned
    drv.axis0.controller.config.vel_gain = 0.1 # Needs tuning

# input("Calibrating, motor will move, press any key to continue: ")

# drv.axis0.requested_state = 3 # AxisState.FULL_CALIBRATION_SEQUENCE

# # Wait for calibration to finish
# while drv.axis0.requested_state == 3:
#     pass

# Set motor and encoder to be pre_calibrated
# drv.axis0.motor.config.pre_calibrated = True
# drv.axis0.encoder.config.pre_calibrated = True

# Can setup
print("CAN Configuration")

drv.can.config.baud_rate = 500000

if controllerType == 'd':
    driveMotorID = input("What is the drive motor can ID? ")
    azID = input("What is the ID of the azimuth encoder? ")

    drv.axis0.config.can.node_id = driveMotorID
    drv.axis1.config.can.node_id = azID

    drv.axis0.config.can.encoder_rate_ms = 10 # send at 100hz
    drv.axis0.config.can.heartbeat_rate_ms = 100 # Send at 10hz

    drv.axis1.config.can.encoder_count_rate_ms = 10 # Send at 100hz
    drv.axis1.config.can.heartbeat_rate_ms = 100 # Send at 10hz
elif controllerType == 'a':
    azMotorID = input("What is the ID of the azimuth motor? ")

    drv.axis0.config.can.node_id = azMotorID

    drv.axis0.config.can.encoder_rate_ms = 10 # Send at 100hz
    drv.axis0.config.can.heartbeat_rate_ms = 100 # Send at 10hz

print("Configuration done, rebooting drive")

try:
    drv.save_configuration()

except Exception as e:
    pass

print("Reacquiring connection...")

drv = odrive.find_any()
print("Reconnected")

input("Running calibration sequence, press any key to continue: ")
drv.axis0.requested_state = 3

# while drv.axis0.requested_state != 1:
#     pass

time.sleep(20)

drv.axis0.motor.config.pre_calibrated = True
drv.axis0.encoder.config.pre_calibrated = True

print("Calibration complete, rebooting drive")

try:
    drv.save_configuration();

except Exception as e:
    pass

print("Sutting down")