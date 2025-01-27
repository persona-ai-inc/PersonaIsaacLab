# Note
drop notes here that might be helpful to use Isaac Sim or Lab



## Environment
[Reference](`https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html#scene-setup`
)


directory structure:
```
my_environment/
    - agents/
        - __init__.py
        - rl_games_ppo_cfg.py
    - __init__.py
    my_env.py
```

an example of an environment is here. It is important to pay attention to the directory structure

`/home/oheidari/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/locomotion/velocity/config/g1`

```
├── agents
│   ├── __init__.py
│   ├── rsl_rl_ppo_cfg.py
│   ├── skrl_flat_ppo_cfg.yaml
│   └── skrl_rough_ppo_cfg.yaml
├── flat_env_cfg.py
├── __init__.py
└── rough_env_cfg.py
```

Here is the scene setup:        
`/home/oheidari/IsaacLab/source/extensions/omni.isaac.lab_assets/omni/isaac/lab_assets/unitree.py`
which gets loaded by one of the config files above. In that directory, I created `Sanctuary.py`

Each env should have its own `configclass` which can include:      
* simulation parameters
* environment scene parameters
* robot parameters
* task-specific parameters.

you can find config in this file:         
`/home/oheidari/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/locomotion/velocity/config/g1/rough_env_cfg.py`

so if you want to set the robot usd/urdf:       
`self.scene.robot = G1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")`



## view usd file
[a quick viewer for usd file](https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html)

download the zip file for Linux and run the following script from the root of extracted directory:

`./scripts/usdview_gui.sh`

you can drag usd file to it, also you can open the gui with a usd file arg:
`./scripts/usdview_gui.sh <path_to_file.usd>`

## Valkyrie robot description
We need the urdf file of the Valkyrie robot. That exists in a separete repo with all the mesh files.    
Download [valkyrie repo](https://github.com/ihmcrobotics/valkyrie.git). Then, look for `val_description` package in that repo.      

In my local system, the urdf exists in:     
`/home/oheidari/java-repos/ihmc-software/repository-group/valkyrie/bin/main/models/val_description/urdf/valkyrie_sim.urdf`
