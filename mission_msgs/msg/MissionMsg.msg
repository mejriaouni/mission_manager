## Priority constants
uint16 HIGH_LEVEL_PRIORITY      = 75
uint16 MEDIUM_LEVEL_PRIORITY    = 50
uint16 LOW_LEVEL_PRIORITY       = 25

# Mission types
uint16 MISSION      = 101
uint16 COMMAND      = 102
uint16 FEEDBACK     = 103

std_msgs/Header header
	
# Mission requested identifier
uint64 mission_id											

# Type of mission - disinfection, navigation, command,   etc
uint64 mission_type

# Mission priority. Only used for command type missions.
uint64 priority

# Mission sequence number. Defines the order in which multiples missions should be put in queue(seq no. 1 is served first). 
# Used in case of array of missions are sent through the service. Otherwise FIFO order
uint64 mission_seq 				
	
# Arguments as JSON string for the action server to perform the task
string mission_args
