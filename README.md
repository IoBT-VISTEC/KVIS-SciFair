# KVIS-SciFair MAZE GAME
Overview
 =
<img src="Images/Maze game final.png" width="1000" height="500">

## Manual
 
[Run code Here](turtlebot3_simulations/turtlebot3_gazebo/src/py)

Run code:
Open Terminal: Ctrl+Alt+T

Terminal 1: 
    - init_world
Terminal 2: 
    - cd work_dir
    - python ttb_template.py

Reset:
    Terminal 2
    - Ctrl+Z
    - reset_ttb    


alias init_world='roslaunch turtlebot3_gazebo multi_turtlebot3.launch'
alias work_dir='~/ttb_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/py'
alias reset_ttb='rostopic pub /ttb_yellow/cmd_vel geometry_msgs/Twist \"linear:
        x: 0.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0\"'
