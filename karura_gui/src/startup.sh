## CLEANUP
#Clean up function
cleanup ()  {
    echo -e "\nShutting down... Deactivating virtual environment."
    # Check if the deactivate function exists (provided by venv)
    if [ -n "$VIRTUAL_ENV" ]; then
        deactivate
    fi
}

#If the script exits for ANY reason, run the cleanup function
trap cleanup EXIT

##RUNNING SCRIPT
#1. Move to project root
cd ../../ || { echo "Directory ./CS2026 not found"; exit 1; }

#2. Activate Python Virtual Env.
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "Virtual environment not found in ./CS2026"
    exit 1
fi

#3. Move to the GUI source directory
cd karura_gui/src || { echo "Directory ./karura_gui/src not found"; exit 1; }

#4. Source the ROS 2 Jazzy enviornment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ROS 2 Jazzy setup file not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

#5. Run the dashboard GUI
echo "Starting Mobility Dashboard..."
python3 -m dashboard.main_mobility