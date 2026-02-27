# Karura Rover Dashboards (PySide6 + ROS 2)

Multi-dashboard control and telemetry GUI for the Karura URC rover, built with **PySide6** and **ROS 2**.

This repository provides three separate dashboards:

- **Mobility Dashboard** – main driving HUD and high-level rover status
- **Science Dashboard** – science sensor telemetry and experiment controls
- **Arm Dashboard** – robotic arm joint states, commands, and camera views

Each dashboard runs as a separate application (typically on separate laptops) but shares a common backend for ROS 2 integration, configuration, and styling.

---

## 1. Features (Initial Scope)

- PySide6-based desktop UI (Qt Widgets)
- ROS 2 `rclpy` backend running in a background thread
- Qt signal/slot bridge from ROS 2 callbacks to GUI widgets
- Shared styles, logging, and configuration across all dashboards
- Separate entry points per dashboard:
  - `mobility` – main driving and battery HUD
  - `science` – science sensor and experiment panel
  - `arm` – manipulator control and feedback
- Modular ROS 2 backend:
  - Role-specific nodes: `MobilityNode`, `ScienceNode`, `ArmNode`, `CommsNode`
  - Generic Qt/ROS bridge + worker thread in `core/`

---

## 2. Repository Layout

```text
karura_gui/
├── README.md
├── requirements.txt
├── run/
│   ├── run_mobility.sh        # helper scripts to launch each dashboard
│   ├── run_science.sh
│   └── run_arm.sh
└── src/
    └── dashboard/
        ├── __init__.py
        ├── main_mobility.py   # entry point for Mobility GUI
        ├── main_science.py    # entry point for Science GUI
        ├── main_arm.py        # entry point for Arm GUI
        ├── core/              # shared Qt/ROS infrastructure
        │   ├── app.py         # Qt app bootstrap (creates bridge + window)
        │   ├── base_bridge.py # BaseROS2Bridge: generic Qt <-> ROS bridge
        │   ├── ros2_worker.py # QThread that runs rclpy.spin_once()
        │   ├── config.py      # topic names, role config, constants
        │   ├── logging_config.py
        │   └── styles.qss     # global Qt stylesheet
        ├── ros_backend/       # ROS 2 nodes for each dashboard role
        │   ├── __init__.py
        │   ├── base_node.py        # BaseDashboardNode with _dispatch()
        │   ├── mobility_node.py    # pubs/subs for mobility topics
        │   ├── arm_node.py         # pubs/subs for arm topics
        │   ├── science_node.py     # pubs/subs for science topics
        │   └── comms_node.py       # pubs/subs for comms/health topics
        ├── mobility/          # Mobility dashboard UI + bridge
        │   ├── __init__.py
        │   ├── bridge.py      # MobilityBridge (Qt signals + MobilityNode)
        │   ├── window.py      # MobilityMainWindow (layouts, widgets)
        │   ├── widgets.py     # reusable mobility-specific widgets
        │   └── view_model.py  # optional data models / adapters
        ├── science/           # Science dashboard UI + bridge
        │   ├── __init__.py
        │   ├── bridge.py      # ScienceBridge (Qt signals + ScienceNode)
        │   ├── window.py      # ScienceMainWindow
        │   ├── widgets.py
        │   └── view_model.py
        └── arm/               # Arm dashboard UI + bridge
            ├── __init__.py
            ├── bridge.py      # ArmBridge (Qt signals + ArmNode)
            ├── window.py      # ArmMainWindow
            ├── widgets.py
            └── view_model.py
```

## 3. How to run
To run main_mobility.py:
1. CD to ./CS2026
2. Run the source .venv/bin/activate
3. CD to ./karura_gui/src
4. Run source /opt/ros/jazzy/setup.bash
5. Run python3 -m dashboard.main_mobility

To simulate backend:
1. Do 1-4 for main_mobility
2. cd to ./dashboard/backend
3. python3 mobility_sim_provider.py

## 4. Libraries to install
- ROS2
- Pyside6
- OpenCV

## 5. How to get camera working locally on Windows
If you want to test it on a webcam on Windows, whether be an integrated one or an external one connected via USB, you need to create a bridge between Windows and WSL.
To do this, install usbipd on your Windows device. Pick the lastest version, though any version of 5.0 or above should be fine.
After that, do

`usbipd list'

This will show the list of devices relating to video footage, including the webcams, which should clearly be displayed under DEVICE. On the same row as the webcam should
be an associated BUSID.

You will then do
`usbipd bind --busid <BUSID>`

and then 
`usbipd attach --busid <BUSID>`

If that doesn't work, do
`usbipd unbind --busid <BUSID>`
`usbipd bind --busid <BUSID> --force`

And restart your device.

After that, it should be connected.


If you have issues still install
- pyyaml (pip install pyyaml) 