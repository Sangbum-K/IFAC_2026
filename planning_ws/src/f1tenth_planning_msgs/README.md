# AE HYU Messages

Custom message definitions for the AE-HYU autonomous vehicle system.

## Quick Start

```bash
# Build
colcon build --packages-select f1tenth_planning_msgs
source install/setup.bash
```

## Message Types

### `Wpnt.msg`
Single waypoint with comprehensive trajectory information:

```yaml
int32 id                    # Waypoint identifier

# Frenet coordinates
float64 s_m                 # Longitudinal position (m)
float64 d_m                 # Lateral position (m)

# Map coordinates
float64 x_m                 # Global X position (m)
float64 y_m                 # Global Y position (m)

# Track boundaries
float64 d_right             # Distance to right boundary (m)
float64 d_left              # Distance to left boundary (m)

# Motion profile
float64 psi_rad             # Heading angle (rad)
float64 kappa_radpm         # Curvature (rad/m)
float64 vx_mps              # Target velocity (m/s)
float64 ax_mps2             # Target acceleration (m/s²)
```

### `WpntArray.msg`
Array of waypoints for trajectory communication:

```yaml
std_msgs/Header header      # Standard ROS header with timestamp
Wpnt[] wpnts               # Array of waypoint messages
```

## Usage

Include as dependency in your `package.xml`:

```xml
<depend>f1tenth_planning_msgs</depend>
```

Include in your `CMakeLists.txt`:

```cmake
find_package(f1tenth_planning_msgs REQUIRED)
ament_target_dependencies(your_target f1tenth_planning_msgs)
```

Use in your code:

```cpp
#include <f1tenth_planning_msgs/msg/wpnt_array.hpp>

// Subscribe to waypoints
auto waypoint_sub = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
    "/planned_waypoints", 10, callback_function);

// Access waypoint data
for (const auto& wpnt : msg->wpnts) {
    double x = wpnt.x_m;
    double y = wpnt.y_m;
    double velocity = wpnt.vx_mps;
    // ... process waypoint
}
```

## Integration

Used by:
- **Lattice Planner**: Publishes planned trajectories as `WpntArray`
- **Controller**: Subscribes to `WpntArray` for path following
- **System Integration**: Common interface for trajectory communication

## Field Descriptions

- **Coordinates**: Both Frenet (s,d) and global (x,y) for flexibility
- **Boundaries**: Track limits for racing applications
- **Motion**: Complete kinematic and dynamic information
- **Trajectory**: Sequential waypoints form continuous path