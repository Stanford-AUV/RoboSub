# Message from GPRMC NMEA String
std_msgs/Header header

string message_id

float64 utc_seconds
string position_status

# Latitude in decimal degrees
float64 lat
# Longitude in decimal degrees
float64 lon

string lat_dir
string lon_dir

float32 speed
float32 track
string date
float32 mag_var
string mag_var_direction
string mode_indicator
