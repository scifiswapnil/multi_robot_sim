## Description
This is a multi-robot stage simulator. You can spawn, as many robots you wish, given you have the computation power. Each robot will be configured with move_base planner and can do path planning, obstacle avoidance, etc. 
To spawn robots, add the new robot in the stage world file and to configure move_base add the robot name alias to the launch file.
The simulation have a `config/config.yaml` file where you can save pre-defined locations and anontate the locations. 

## References (ROS resources)
- [robot setup](http://wiki.ros.org/navigation/Tutorials/RobotSetup)
- [move base](http://wiki.ros.org/move_base) 
- [tuning guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
  
## Node details :
- mission_controller node
  - Is a controller designed to control the behaviour of the robot. Basically, its a queue based mission controller, that stores and controls the execution of the goals sent to the robot. 
  - `mission_controller_digester` is a service that is used to send the goals to the robot. You can send the predefined location goals as defined in the `config/config.yaml`. Check the config.yaml for detials 
  - `mission_controller_state` is service that controls the queue execution. Like, if you want to stop the queue execution, meaning stop the robot performing a sequence of goals, you can send "stop" via this service. list of possible states that can be sent are as follows :  
      - stop - stop the robot, but keep the goals in the queue.
      - cancel - cancel the current goal, continue with next.
      - start - start the execution, by default the queue is always executing. 
      - clear - clears the future goals in the queue. 
  - example : 
    - to send the robot to parking location one, call the following service from the terminal 
  ```
  rosservice call /robot_0/mission_controller_digester "goals: data: 'P1'" 
  ```
    - to control the state of the robot mission queue, call the following service from the terminal 
  ```
  rosservice call /robot_0/mission_controller_state "state: data: 'start'" 
  ```
- mission_controller_client
  - Is a client node for the above controller node. Basically reads a sequence of codes from a text file in `worksheets` folder, having the same name as the robots name. 
  - reads the sequence of goals, one by one and send it to the mission_controller.