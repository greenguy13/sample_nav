GOAL: Make the move_base 

What to place here?
Launch file:
mission.launch
robots launchfile: nodes.launch, robot_sim2.launch

mission.launch 

robots:
nodes.launch - name prefix, move base config, fake localization
robot_sim2.launch - launches 2 robots with nodes.launch as the configuration 


I think I know where the problem is now: base_link topic does not exist. This should be base_scan; 
or the other way around.
    