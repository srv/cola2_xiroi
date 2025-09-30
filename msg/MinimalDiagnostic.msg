Header header

# Possible levels of operations
byte OK=0    # Sensor working correctly
byte WARN=1  # Sensor has problem
byte ERROR=2 # There has been an error
byte STALE=3 # Waiting for init
byte OFF=4   # Sensor is disabled

byte status

# Max 63 bytes for SENDIM (Acoustic link)
string name
string description