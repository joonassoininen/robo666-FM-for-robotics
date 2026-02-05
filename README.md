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
   Python-version: 3.11.*

## Models and dataset

Our trained models and dataset can be found from huggingface:  
  * **Models**:  
     * Trained X-VLA model: [https://huggingface.co/Rikuhaapala/xvla-franka-20000steps](https://huggingface.co/Rikuhaapala/xvla-franka-20000steps)  
     * Trained SmolVLA model: [https://huggingface.co/Joonassoininen/robo666_smolvla](https://huggingface.co/Joonassoininen/robo666_smolvla)  

  * **Datasets**:  
     * X-VLA training data: [https://huggingface.co/datasets/Rikuhaapala/robo666_V2](https://huggingface.co/datasets/Rikuhaapala/robo666_V2)  
     * SmolVLA training data: [https://huggingface.co/datasets/Joonassoininen/robo666](https://huggingface.co/datasets/Joonassoininen/robo666)  
   

## Usage (reocrding)
Open terminal and source to the root directory of this project and paste:
```bash
source /opt/ros/humble/setup.bash  
export ROS_DOMAIN_ID=100  
colcon build  
source install/setup.bash  
ros2 launch franka_gazebo robot.launch.py
```
Open a second terminal and paste:
```bash
source /opt/ros/humble/setup.bash  
export ROS_DOMAIN_ID=100
ros2 run joy joy_node
```

Open a third terminal, source to the crisp_py folder of this project and paste:
```bash
pixi shell -e humble  
export PYTHONPATH=<insert_local_path_to_the_src_folder>:$PYTHONPATH
python3 ~/robo666_project/src/crisp_py/examples/record_teleop.py  
```
Open a fourth terminal, source to the crisp_gym folder of this project and paste: 
```bash
source scripts/configure.sh 
pixi install
pixi shell -e humble-lerobot
python -c "import crisp_gym"
python3 scripts/record_lerobot_format_single_robot.py   --repo-id <insert_file_where_you_want_to_save_dataset>
```
When you run record_lerobot_format_single_robot.py it first asks follower robot namespace. Put empty namespace on that question by hitting enter. then it asks follower robot configuration, we used option 6 where we manually defined cameras and gripper. After entering 6, robot goes to home position.

Then open a fifth terminal, source to the crisp_py folder of this project and paste: 
```bash
pixi shell -e humble  
export PYTHONPATH=<insert_local_path_to_the_src_folder>:$PYTHONPATH
ros2 control switch_controllers --activate gripper_effort_controller --strict
```
Then open a sixth terminal and paste: 
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=100
```
If you want to start/stop recording, enter this to the sixth terminal:
```bash
ros2 topic pub --once /record_transition std_msgs/msg/String "{data: record}"
```
If you want to save rercording:
```bash
ros2 topic pub --once /record_transition std_msgs/msg/String "{data: save}"
```
If you want to delete last recording:
```bash
ros2 topic pub --once /record_transition std_msgs/msg/String "{data: delete}"
```
After you press record you can control the robot and crisp_gym saves robot and camera data. 

## Usage (deploy policy)

Open terminal and source to the root directory of this project and paste:
```bash
source /opt/ros/humble/setup.bash  
export ROS_DOMAIN_ID=100  
colcon build  
source install/setup.bash  
ros2 launch franka_gazebo robot.launch.py
```

Open a second terminal, source to the crisp_gym folder of this project and paste: 
```bash
source scripts/configure.sh 
pixi install
pixi shell -e humble-lerobot
python -c "import crisp_gym"
python3 scripts/deploy_policy.py \ --path Rikuhaapala/xvla-franka-20000steps \ --repo-id <insert_repo_where_you_want_to_store_evaluation>
```
Open a third terminal and paste, source to the crisp_py folder of this project and paste:
```bash
pixi shell -e humble  
export PYTHONPATH=<insert_local_path_to_the_src_folder>:$PYTHONPATH
ros2 control switch_controllers --activate gripper_effort_controller --strict
```
Open a fourth terminal and paste: 
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=100
```
In the fourth terminal, you can start robot/model to perform the task by entering: 
```bash
ros2 topic pub --once /record_transition std_msgs/msg/String "{data: record}"
```
**Training
Information about how to train models can be found from huggingface lerobot page [https://huggingface.co/docs/lerobot/xvla](https://huggingface.co/docs/lerobot/xvla)



