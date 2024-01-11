# Platoon System Simulation 

The Platoon System Simulation is a program that simulates the behavior of leading and following vehicles in a Platoon System environment. It allows you to simulate the interaction between multiple vehicles and observe their positions and speeds over time.

## Leading Vehicle

The Leading Vehicle represents the front vehicle in the convoy. It is responsible for setting the pace and providing information to the Following Vehicles. The `LeadingVehicle` class, defined in the `lead.h` header file, contains the necessary structures and functions for simulating the behavior of the leading vehicle. It has attributes such as ID, position, speed, and a server socket for communication.

## Following Vehicles

The Following Vehicles represent the vehicles that follow the Leading Vehicle in the convoy. The `FollowingVehicle` class, defined in the `follow.h` header file, provides functionalities for connecting to the Leading Vehicle, sending and receiving messages, and controlling the position and speed based on the received information. It has attributes such as ID, position, speed, server socket, target distance, and PID controller gains (Kp, Ki, Kd).

The Following Vehicles utilize the socket-based communication mechanism to establish a connection with the Leading Vehicle. They continuously send follower messages to the Leading Vehicle, containing their current position, speed, and connection status. They also receive state information from the Leading Vehicle, including the target speed. Based on the received information, the Following Vehicles adjust their speed and position to maintain a predefined distance from the Leading Vehicle.

The system provides a command-line interface for configuring the Following Vehicles, such as setting the initial speed, initial position, target distance, and PID controller gains. It also allows the user to monitor the state of the Following Vehicles and manually terminate the simulation.

To ensure smooth operation, the system utilizes separate threads for receiving messages from the Leading Vehicle, sending follower messages continuously, and handling user input while monitoring the state of the Following Vehicles. Terminal settings are adjusted to enable non-canonical mode and disable input buffering for responsive user interaction.

## System Requirements

- C++ compiler supporting C++11 or higher
- Operating system: Windows, macOS, or Linux (Note: This system is only tested on Linux)

## Getting Started

To run the simulation, follow these steps:

1. Clone or download the repository to your local machine.

2. Navigate to the project directory.

3. Compile the source code using the following command:
- For leading vehicle:
   ```bash
   g++ -std=c++11 lead.cpp -o lead
   
- For following vehicle:
   ```bash
   nvcc follow.cu -o follow

4. Run simulation:
- Initial the leading vehicle, start server, waiting from connections with following vehicles
    ```bash
    ./lead

- One following vehicle will be created and send signal to the leading vehicle each time the follow file excuted    
    ```bash
    ./follow --initspeed [initspeed] --initposition [initposition] --distance [distance] --id [id]
