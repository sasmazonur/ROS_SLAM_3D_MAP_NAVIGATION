### This folder contains the roslaunch scripts for starting the navigation.

/launch/amcl.launch

![Image of AMCL](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/Amcl-map-localization.png)

Amcl is a probabilistic localization system for a robot moving in 2D.
It implements the adaptive Monte Carlo localization approach , which uses a
particle filter to track the pose of a robot against a known map.


/Params
* base_local_planner_params.yaml              # The parameter of the speed command to the robot
* costmap_common_params_burger.yaml           # The parameter of costmap configuration consists
* costmap_common_params_waffle.yaml           # The parameter of costmap configuration consists
* costmap_common_params_waffle_pi.yaml        # The parameter of costmap configuration consists
* dwa_local_planner_params.yaml               # The parameter of the speed command to the robot
* global_costmap_params.yaml                  # The parameter of the global area motion planning
* local_costmap_params.yaml                   # The parameter of the local area motion planning
* move_base_params.yaml                       # parameter setting file of move_base that
                                              supervises the motion planning.
