## Setup Instructions

### 1. Create a Workspace

```bash
mkdir -p psdk_ws/src && cd psdk_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/bitcurious/dji_rc_controller.git
```

### 3. Build the Package

Navigate to your workspace and build the package:

```bash
cd ~/psdk_ws
colcon build --packages-select dji_rc_controller
```

### 4. Run the Controller Node

```bash
ros2 run dji_rc_controller rc_controller
```

### 5. Select Control Mode

When the program is running, you will be prompted to enter a control mode:

- `0` = Attitude Control
- `1` = Velocity Body
- `2` = Velocity Ground Absolute Yaw
- `3` = Velocity Ground Yaw Rate
- `q` = Quit

Simply enter the desired mode number to test different control options.

--- 
