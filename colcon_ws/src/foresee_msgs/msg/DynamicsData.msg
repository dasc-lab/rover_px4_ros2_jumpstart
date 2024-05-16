# combined data (VICON and ref) in NED frame
# Input to Gaussian Process.
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint64 timestamp # time since system start (microseconds)

# VICON measurement NED local world frame
float32[3] pos # in meters
float32[3] vel # in meters/second
float32[3] acc # in meters/second^2


# NED local world frame
float32[3] pos_ref # in meters
float32[3] vel_ref # in meters/second
float32[3] acc_ref # in meters/second^2