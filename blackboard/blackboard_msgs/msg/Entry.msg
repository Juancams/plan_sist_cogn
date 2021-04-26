uint8 BOOL_TYPE=0
uint8 STRING_TYPE=1
uint8 FLOAT_TYPE=2
uint8 POSESTAMPED_TYPE=3
uint8 OCTOMAP_TYPE=4

uint8 type
string parent_key
string key

bool bool_entry
string string_entry
float32 float_entry
geometry_msgs/PoseStamped posestamped_entry
octomap_msgs/Octomap octomap_entry