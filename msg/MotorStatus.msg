# 50Hz feedback message for controls purposes
Header header

# Current flowing in the motors (A)
float64 motor_current

# Output stage, as a proportion of full (-1..1)
float64 motor_power

# Electrical power supply to the driver (V, A)
float64 supply_voltage
float64 supply_current

# Measured temperatures (C)
# Motor temp is processed from a thermal sensor connected to analog input 1.
float32 motor_temperature

# Channel temp is the temperature of the FETs. This is reported by the controller.
float32 channel_temperature
