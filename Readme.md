
## 🚀 Overview

- **Communication Hub**: gRPC-based service for robot-to-hub communication using Modbus protocol
- **Delivery Hub**: Modbus server simulation for package handling and bin management
- **Fleet Management**: ROS 2-based robot fleet coordination and task management

## Directory Structure

```
.
├── communication_hub
├── delivery_hub
├── extras
└── fleet
    └── src
        ├── fleet_launcher
        ├── path_planner
        ├── robot_msgs
        └── task_manager
```

## 🛠️ Components

### Communication Hub (Rust)
- **Purpose**: Central communication service for robot-delivery hub coordination
- **Features**:
  - Robot availability checking
  - Package acceptance verification

### Delivery Hub (Python)
- **Purpose**: Simulates delivery hub operations (mimicing PLC)
- **Features**:
  - Multi-slave Modbus server.
  - GUI visualization

### Fleet Management (ROS 2- foxy)
- **Purpose**: Robot fleet coordination
- **Packages**:
  - `fleet_launcher`: Launch all the nodes( ToDo) and contains robot_visulizer and package simulator.
  - `path_planner`: A* path planner 
  - `task_manager`: Task allocation and scheduling
  - `robot_msgs`: Custom message definitions
  - `odom_publisher`: Odometry data handling and robot motion simulator.

### Extras
- **Purpose**: configurations and other files



### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd lexxpluss
   ```

2. **Build Communication Hub**
   ```bash
   cd communication_hub
   cargo build --release
   ```

3. **Install Python dependencies**
   ```bash
   cd delivery_hub
   pip install pymodbus PyQt5
   ```

4. **Build ROS 2 packages**
   ```bash
   cd fleet
   colcon build
   source install/setup.bash
   ```

### Running the System

1. **Start Delivery Hub Server**
   ```bash
   cd delivery_hub
   python hub_server.py 
   ```

2. **Launch Communication Hub**
   ```bash
   cd communication_hub
   cargo run
   ```

3. **Run Visualizer**
   ```bash
   cd delivery_hub
   python del_hub_visualiser.py
   ```

## Other documentation
 [Documentation](https://www.notion.so/220ae74bd23880d28e24f494836fe45b?v=225ae74bd2388016b8c9000c03b32c10)