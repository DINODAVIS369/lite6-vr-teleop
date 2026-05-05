# 🦾 Lite6 VR Teleoperation (Mirror Mode)

This project enables real-time, 1:1 Cartesian teleoperation of a Lite6 robot arm using a Meta Quest 3 headset.

## 🚀 Quick Start (Tomorrow's First Steps)

1. **Power on the Robot** (Physical or Simulation).
2. **Start the Quest 3 App** (Hand Tracking Streamer).
3. **Run the Launch Command**:
   ```bash
   source install/setup.bash
   ros2 launch mylit6 lite6_vr_gazebo.launch.py
   ```

## 🎮 How it Works
- **Bridge**: `quest_bridge.py` listens for raw UDP packets on Port 5005.
- **Tracking**: Currently set to **Wrist Tracking** (most stable).
- **Mirroring**: 1:1 Position and 6-DOF Rotation mapping.
- **Safety**: Velocity is capped at 1.5 m/s for rapid but safe response.

## 🛠️ Key Technical Details
- **Coordinate Map**: 
  - ROS X = Unity Z (Forward)
  - ROS Y = -Unity X (Left)
  - ROS Z = Unity Y (Up)
- **Networking**: Uses `SO_REUSEADDR` to prevent "Port already in use" errors on restart.
- **Filtering**: Low-pass alpha filter (0.8) for responsive but clean motion.

## 💾 Project State
- **Git**: Initialized with `.gitignore` (build/log/install ignored).
- **Branch**: `main`
- **Latest Commit**: 6-DOF Mirroring Implementation.

---
*Created with 🦾 Antigravity AI Assistant*
