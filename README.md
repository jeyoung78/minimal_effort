# Minimal Effort
Dockerized ROS2 repo for a 4-arm ball balancing platform. Developed as a part MTE380 course project in a group of four. 

https://github.com/user-attachments/assets/7025b53a-dd6f-4625-b840-a4bffec2d5b9

## Summary
The project aimed to create a compact, aesthetic, and robust platform capable of stabilizing a ping-pong ball and following predefined trajectories. The Stewart platform, a parallel manipulator known for its precision and reliability, was selected to demonstrate principles of mechanical design, control systems, and advanced kinematics.

The design process emphasized simplicity, modularity, and aesthetics. Key features include a square aluminum base for ease of integration, servo motors for precise control, and an acrylic platform. A single overhead camera and real-time image processing algorithms were used to track the ball's position, providing feedback for stabilization and trajectory planning.

The project used ROS2 as the software framework for modularity and reliability, while inverse kinematics algorithms were developed to handle the platform’s unique four-arm configuration. A PID controller-based path-planning algorithm guided the platform's movements, achieving rapid adjustments to maintain balance and stability. The perception system relied on a color filter to detect the ball, avoiding complex and computationally expensive alternatives like Kalman Filters or perspective transformations.

Despite achieving all functional goals, challenges such as overheating of the microcontroller during prolonged use highlighted areas for future improvement. The project successfully demonstrated the effectiveness of combining aesthetic design with software-driven performance, offering a practical solution to real-world stabilization problems. This report includes recommendations for further refinement and explores potential patent opportunities for the innovative software architecture.

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
