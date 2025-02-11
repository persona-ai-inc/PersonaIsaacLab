[Reference](https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/)

We need two phases:             
rollout phase and the learning phase

**rollout**: we need to run the simulation (based on the current policy) for a number of steps to collect data for training

The agent samples actions for the N environments and continue to step them for a fixed number of M steps. During these M steps, the agent continues to append relevant data in an empty list data



## Env Configuration
`velocity_env_cfg.py` in managerbased env is the file to change some env settings