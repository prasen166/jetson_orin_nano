#!/bin/bash

# Deactivate conda (if active)
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
  echo "Deactivating Conda environment: $CONDA_DEFAULT_ENV"
  conda deactivate
fi

commands=(
  "cd /home/tihan-6g/gps/ws_livox && source devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch"
  "cd /home/tihan-6g/gps/fastlio && source devel/setup.bash && roslaunch fast_lio mapping_mid360.launch"
  "cd /home/tihan-6g/gps && source devel/setup.bash && roslaunch fastlio_mavros_bridge fastlio_mavros_integration.launch"
)

titles=(
  "Livox"
  "Fast_LIO"
  "MAVROS"
)

sleep_durations=(2 3 3)

for i in "${!commands[@]}"; do
  gnome-terminal --tab --title="${titles[i]}" -- bash -c "
    echo 'Running: ${titles[i]}';
    
    # Ensure conda is not active inside new terminal
    if [[ -n \"\$CONDA_DEFAULT_ENV\" ]]; then
      conda deactivate
    fi

    ${commands[i]};
    exec bash
  "
  
  sleep "${sleep_durations[i]}"
done
