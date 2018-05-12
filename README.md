# URDF Human model for ROS + MoveIt `human_moveit_config`
This is a Human model using moveit with 32DOF URDF model.
This package is undocumented and no longer maintained. It has been tested on ROS Kinetic.

If you came accross this repo from a search engine, you more likely only want to import the [URDF of the human model](urdf/human.urdf) in your own setup, e.g. by providing it to the [MoveIt setup assistant](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) that will generate your own (up to date!) copy of the package `human_moveit_config`. From that point you'll be able to plan trajectories of hands and feet using the regular [MoveIt planning stack](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html).

All other code in this repository are made for customization of the URDF according to people's morphology, involving Optitrack and Kinect sensors. [Read additional info](https://github.com/baxter-flowers/human_moveit_config/issues/2#issuecomment-286433640).
