std_msgs/Header header

# Normalized detected-frequency level per hydrophone (0..1), peak since the
# previous message. Front hydrophones are 0 and 3; back are 1 and 2.
float32[4] levels

# Latched pinger direction: true = front, false = back (sticky, back at boot).
bool front
