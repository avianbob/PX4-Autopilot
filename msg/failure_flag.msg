#####################################################custom failure_flag.msg file###################################################################

# Timestamp for the message (px4/uint64_t)
uint64 timestamp

# Flag indicating if a failure is detected
bool failure_detected    # True if a failure is detected

# Index of the failed motor (-1 if not applicable)
uint8 failed_motor_index

# Type of failure (0 = none, 1 = motor, etc.)
uint8 failure_type

################################################################################################################################################