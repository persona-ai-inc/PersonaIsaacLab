# Move Joints


## How to move a rigid body in Sim
Reference: `/home/oheidari/IsaacLab/source/standalone/tutorials/01_assets/run_rigid_object.py`      

Set the pose and velocity of a rigid body can be done with the following commands:      

```
cone_object.write_root_link_pose_to_sim(root_state[:, :7])
cone_object.write_root_com_velocity_to_sim(root_state[:, 7:])
```

In the same file, you can see how to make a rigid object and its properties.


The two main sections are:      
`Design Scene` and `Run Simulator`


To set joint pos, you can look at `run_articulation.py`. We can use 
`write_joint_state_to_sim` to do that.