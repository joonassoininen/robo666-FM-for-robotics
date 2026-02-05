## Acknowledgements & Credits

This project utilizes and builds upon the following open-source frameworks: 

* **LeRobot**: A state-of-the-art framework for deep learning and robotics by Hugging Face.
    * Source: [https://github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)
    * License: Apache 2.0
* **CRISP**: (CRISP - Compliant ROS2 Controllers for Learning-Based Manipulation Policies).
    * Includes: `crisp_py`, `crisp_gym`, and `crisp_controllers`.
    * Source:
         * crisp_py: [https://github.com/utiasDSL/crisp_py](https://github.com/utiasDSL/crisp_py)
         * crisp_gym: [https://github.com/utiasDSL/crisp_gym](https://github.com/utiasDSL/crisp_gym)
         * crisp_controllers: [https://github.com/utiasDSL/crisp_controllers](https://github.com/utiasDSL/crisp_controllers)

## Requirements
   operating system: Ubuntu 22.04  
   ROS2-version: Humble  
   Python-version: <3.11

## Usage (reocrding)
Open terminal and source to the root directory of this project and past:
```bash
source /opt/ros/humble/setup.bash  
export ROS_DOMAIN_ID=100  
colcon build  
source install/setup.bash  
ros2 launch franka_gazebo robot.launch.py

Open new terminal, source to the crisp_py folder of this project and past:
```bash
pixi shell -e humble  
export PYTHONPATH=<insert_local_path_to_the_src_folder>:$PYTHONPATH
python3 ~/robo666_project/src/crisp_py/examples/record_teleop.py
