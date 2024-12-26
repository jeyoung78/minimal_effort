# Minimal Effort
Dockerized ROS2 repo for a 4-arm ball balancing platform.

https://github.com/user-attachments/assets/7025b53a-dd6f-4625-b840-a4bffec2d5b9


## To Use
1. Clone the repo:
``` bash
git clone git@github.com:Edwardius/minimal_effort.git
```

2. Set the modules you want to run in the `mind-config.sh`. In the case of this repo, it is just stewart.

3. Run docker compose using the provided script:
```bash
./mind up
```

You are all set! You can view more developer tools and tips at https://wiki.watonomous.ca/autonomous_software/monorepo_infrastructure

All source code is located under `src` and are all rosnodes.
