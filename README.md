# GTEC FTM - ROS2 Package

ROS2 package for reading data from ESP32S2 FTM (Fine Timing Measurement) devices via serial port and publishing to ROS2 topics.

## Overview

This package provides ROS2 nodes to interface with ESP32S2 FTM Tag Reader devices. It reads ranging data from the serial port and publishes it as ROS2 messages.

## Node Versions

This package includes two versions of the FTM Tag Reader node, designed to work with different ESP32S2 firmware configurations:

### ESP32S2FTMTagReader (Standard Version)
- **Purpose**: Basic FTM tag reader for standard ranging measurements
- **CSV Format**: `anchorId,rtt_est,rtt_raw,dist,numFrames,frame1_data,frame2_data,...`
- **Message Type**: `gtec_msgs/ESP32S2FTMRanging`
- **Use Case**: Standard FTM ranging applications where basic distance estimation is sufficient

### ESP32S2FTMTagReaderExtra (Extended Version)
- **Purpose**: Enhanced FTM tag reader with additional distance estimation
- **CSV Format**: `anchorId,rtt_est,rtt_raw,dist,own_dist,numFrames,frame1_data,frame2_data,...`
- **Message Type**: `gtec_msgs/ESP32S2FTMRangingExtra`
- **Additional Field**: `own_dist` - provides an additional distance estimate calculated by the device
- **Use Case**: Applications requiring multiple distance estimation methods or enhanced accuracy analysis

**Key Difference**: The Extra version includes an additional `own_dist` field in the CSV data stream, which provides a secondary distance estimate. This requires compatible ESP32S2 firmware that outputs the extended CSV format.

## Features

- Two node variants for different ESP32S2 firmware configurations
- ROS2 launch files for easy deployment
- Configurable serial port parameter
- Real-time FTM ranging data processing
- Frame-level data parsing (RTT, RSSI, timestamps)

## Dependencies

- ROS2 (tested with ROS2 Humble/Iron/Jazzy)
- Python 3
- `pyserial` library
- `gtec_msgs` package (custom message definitions)

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
# Install Python serial library
pip3 install pyserial

# Or using apt (if available)
sudo apt install python3-serial
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Basic FTM Tag Reader

```bash
# Launch with default settings
ros2 launch gtec_ftm esp32s2ftmtagreader_launch.py

# Launch with custom serial port
ros2 launch gtec_ftm esp32s2ftmtagreader_launch.py serial:=/dev/ttyUSB1
```

### Extended FTM Tag Reader

```bash
# Launch with default settings
ros2 launch gtec_ftm esp32s2ftmtagreaderextra_launch.py

# Launch with custom serial port
ros2 launch gtec_ftm esp32s2ftmtagreaderextra_launch.py serial:=/dev/ttyUSB1
```

### Running Nodes Directly

```bash
# Run basic reader
ros2 run gtec_ftm ESP32S2FTMTagReader --ros-args -p serial:=/dev/ttyUSB0

# Run extended reader
ros2 run gtec_ftm ESP32S2FTMTagReaderExtra --ros-args -p serial:=/dev/ttyUSB0
```

## Topics

Both nodes publish to the `/gtec/ftm/` topic but with different message types:

- **Standard Version**: Publishes `gtec_msgs/ESP32S2FTMRanging` messages
- **Extended Version**: Publishes `gtec_msgs/ESP32S2FTMRangingExtra` messages (includes additional `own_est` field)

**Note**: Choose the appropriate version based on your ESP32S2 firmware output format. The Extended version is only compatible with firmware that outputs the additional distance estimate field.

## Parameters

- `serial` (string, default: "/dev/ttyUSB0"): Serial port device path

## Message Format

### Standard Version (ESP32S2FTMTagReader)
Parses CSV data in the format: `anchorId,rtt_est,rtt_raw,dist,numFrames,frameData...`

Published message (`gtec_msgs/ESP32S2FTMRanging`) contains:
- `anchorid`: Anchor identifier
- `rtt_est`: Processed RTT estimate
- `rtt_raw`: Raw RTT measurement
- `dist_est`: Distance estimate (in meters)
- `num_frames`: Number of measurement frames
- `frames[]`: Array of frame data (RTT, RSSI, T1-T4 timestamps)

### Extended Version (ESP32S2FTMTagReaderExtra)
Parses CSV data in the format: `anchorId,rtt_est,rtt_raw,dist,own_dist,numFrames,frameData...`

Published message (`gtec_msgs/ESP32S2FTMRangingExtra`) contains:
- `anchorid`: Anchor identifier
- `rtt_est`: Processed RTT estimate
- `rtt_raw`: Raw RTT measurement
- `dist_est`: Primary distance estimate (in meters)
- `own_est`: Secondary distance estimate (in meters) - **Extra field**
- `num_frames`: Number of measurement frames
- `frames[]`: Array of frame data (RTT, RSSI, T1-T4 timestamps)

## License

MIT License - see LICENSE file for details.

## Authors

- Valentin Barral (valentin.barral@udc.es)
- Group of Electronic Technology and Communications, University of A Coruña

