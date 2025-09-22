# COLA2_xiroi

xiroi AUV specific files for the COLA2 software architecture.

Summary to understand the structure:

- cola2_core contains the generic nodes (common for most robots), we will use its '.launch' files as a reference, but we won't modifie them.

- cola2_xiroi is a specific package for our robot.

in config/ we will have different .yaml files with the parameters that we want to load to the robot

we will have 3 main .launch files:
- configs.launch: will load all .yaml files from cola2_xiroi/config
- core.launch will load all the differente nodes of the robot (safety.launch, controll.launch,...) nodes the will be defined in cola2_xiroi/launch/submodules, and will call the generic nodes of cola2_core with its own configurations (specified in its yaml file)
- start.launch will initialize all the robot (including this 2 big .launch)