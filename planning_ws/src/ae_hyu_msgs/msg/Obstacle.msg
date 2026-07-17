# Single obstacle information
int32 id
geometry_msgs/Point position  # Global coordinates (x, y, z)
float64 s_center
float64 d_center
float64 s_start
float64 s_end
float64 d_right               # Defined as right bound of the obstacle
float64 d_left                # Defined as left bound of the obstacle
float64 vs
float64 vd
float64 s_var
float64 d_var
float64 vs_var
float64 vd_var
float64 size
float64 theta                 # Only defined for static (rad)
bool is_static            
bool is_visible
int32 curr_lap
