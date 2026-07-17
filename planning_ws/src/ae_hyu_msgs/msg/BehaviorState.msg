std_msgs/Header header

# -----------------------
# Time trial mode states
# -----------------------
uint8 AVOID = 0
uint8 GLOBAL = 1
uint8 state

# -----------------------
# H2H mode states
# -----------------------
# Time state
uint8 FIRST_LAP   = 0
uint8 STATIC_ON   = 1
uint8 STATIC_OFF  = 2
uint8 time_state

# Section state
uint8 SECTION_1 = 1
uint8 SECTION_2 = 2
uint8 SECTION_3 = 3
uint8 SECTION_4 = 4
uint8 SECTION_5 = 5
uint8 section_state

# -----------------------
# Extra info
# -----------------------
uint32 lap_count
bool need_slowdown
bool start_opposite
bool b_is_in_pitstop_area