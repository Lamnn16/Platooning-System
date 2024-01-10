#include "follow.h"
#include <cuda_runtime.h>

FollowingVehicle::FollowingVehicle(int id, double initialSpeed, double initialPosition, double targetDistance, double Kp, double Ki, double Kd)
    : id(id), position(initialPosition), speed(initialSpeed), serverSocket(0), targetDistance(targetDistance),
      Kp(Kp), Ki(Ki), Kd(Kd), integralError(0.0), previousError(0.0) {}

int FollowingVehicle::getId() const
{
    return id;
}

double FollowingVehicle::getPosition() const
{
    return position;
}

double FollowingVehicle::getSpeed() const
{
    return speed;
}

void FollowingVehicle::setPosition(double newPosition)
{
    position = newPosition;
}

void FollowingVehicle::setSpeed(double newSpeed)
{
    speed = newSpeed;
}

void FollowingVehicle::connectToLeader(const std::string &ipAddress)
{
    // Create client socket and connect to the leading vehicle
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        std::cerr << "Error creating client socket" << std::endl;
        return;
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);

    if (inet_pton(AF_INET, ipAddress.c_str(), &(serverAddress.sin_addr)) <= 0)
    {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return;
    }

    if (connect(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "Connection failed" << std::endl;
        return;
    }
    std::cout << "Connected to the leading vehicle!" << std::endl;
}

void FollowingVehicle::sendFollowerMessage()
{
    // Send follower message
    Message followerMessage;
    followerMessage.senderId = id;
    followerMessage.position = position;
    followerMessage.speed = speed;
    followerMessage.isConnected = true;

    if (send(serverSocket, &followerMessage, sizeof(followerMessage), 0) < 0)
    {
        std::cerr << "Error sending follower message" << std::endl;
    }
}

void FollowingVehicle::sendFollowerMessagesContinuously(int interval)
{
    while (true)
    {
        sendFollowerMessage();
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}

// CUDA kernel function for calculating speed and distance
__global__ void calculateSpeedAndDistance(double* deviceSpeed, double* devicePosition, double leaderPosition,double targetSpeed, double targetDistance, double Kp, double Ki, double Kd)
{
    int index = blockIdx.x * blockDim.x + threadIdx.x;

    // Calculate speed
    double error = targetSpeed - deviceSpeed[index];
    double integralError = 0.0; // Assume integralError and previousError are stored in device memory
    double previousError = 0.0;
    double derivativeError = error - previousError;
    double controlSignal = Kp * error + Ki * integralError + Kd * derivativeError;
    deviceSpeed[index] += controlSignal;

    // Calculate distance
    double distanceError = targetDistance - (leaderPosition - devicePosition[index]);
    double positionControlSignal = Kp * distanceError;
    devicePosition[index] -= positionControlSignal;
}

void FollowingVehicle::receiveStateFromLeader()
{
    Message message;

    if (recv(serverSocket, &message, sizeof(message), 0) < 0)
    {
        std::cerr << "Error receiving state from leader" << std::endl;
    }
    else
    {
        // Use the received speed as the targetSpeed
        double targetSpeed = message.speed;
        double leaderPosition = message.position;

        // Allocate device memory for speed and position
        double* deviceSpeed;
        double* devicePosition;
        cudaMalloc((void**)&deviceSpeed, sizeof(double));
        cudaMalloc((void**)&devicePosition, sizeof(double));

        // Copy speed and position data to device memory
        cudaMemcpy(deviceSpeed, &speed, sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(devicePosition, &position, sizeof(double), cudaMemcpyHostToDevice);

        // Call the CUDA kernel for speed and distance calculation
        calculateSpeedAndDistance<<<1, 1>>>(deviceSpeed, devicePosition, leaderPosition, targetSpeed, targetDistance, Kp, Ki, Kd);

        // Copy the results back to the host
        cudaMemcpy(&speed, deviceSpeed, sizeof(double), cudaMemcpyDeviceToHost);
        cudaMemcpy(&position, devicePosition, sizeof(double), cudaMemcpyDeviceToHost);

        // Free device memory
        cudaFree(deviceSpeed);
        cudaFree(devicePosition);
    }
}

void FollowingVehicle::receiveMessageFromLeader()
{
    while (true)
    {
        receiveStateFromLeader();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add a delay between receiving messages
    }
}

void FollowingVehicle::printState()
{
    std::cout << "Following Vehicle ID: " << id;
    std::cout << "  Position: " << position;
    std::cout << "  Speed: " << speed << std::endl;
}

void FollowingVehicle::stopServer()
{
    close(serverSocket);
}

// Function to set the terminal in non-canonical mode and disable input buffering
void setNonCanonicalMode()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// Function to restore the terminal settings
void restoreTerminalSettings()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// Function to check if there is keyboard input available
bool isKeyPressed()
{
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(STDIN, &readSet);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int selectResult = select(STDIN + 1, &readSet, NULL, NULL, &timeout);
    if (selectResult == -1)
    {
        std::cerr << "Error in select" << std::endl;
        return false;
    }

    return FD_ISSET(STDIN, &readSet);
}

int main(int argc, char *argv[])
{
    // Default values
    double initialSpeed = 0.0;
    double targetDistance = 10.0;
    double Kp = 0.1;
    double Ki = 0.01;
    double Kd = 0.01;
    int id = 2;
    double initialPosition = 10.0;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--id" && i + 1 < argc)
        {
            id = std::stoi(argv[i + 1]);
        }
        if (arg == "--initspeed" && i + 1 < argc)
        {
            initialSpeed = std::stod(argv[i + 1]);
        }
        else if (arg == "--initposition" && i + 1 < argc)
        {
            initialPosition = std::stod(argv[i + 1]);
        }
        else if (arg == "--distance" && i + 1 < argc)
        {
            targetDistance = std::stod(argv[i + 1]);
        }
    }

    FollowingVehicle follower(id, initialSpeed, initialPosition, targetDistance, Kp, Ki, Kd);

    std::string ipAddress = "127.0.0.1"; // IP address of the leading vehicle

    // Connect to the leader
    follower.connectToLeader(ipAddress);

    // Start a separate thread to receive messages from the leader continuously
    std::thread receiveThread(&FollowingVehicle::receiveMessageFromLeader, &follower);
    receiveThread.detach(); // Detach the thread to let it run independently

    // Start a separate thread to continuously send follower messages
    std::thread followerMessageThread(&FollowingVehicle::sendFollowerMessagesContinuously, &follower, 1000); // Send follower message every 1 second
    followerMessageThread.detach();                                                                          // Detach the thread to let it run independently

    setNonCanonicalMode(); // Set the terminal in non-canonical mode
    while (true)
    {
        follower.printState();

        // Check for input
        if (isKeyPressed())
        {
            char c;
            std::cin.get(c);
            if (c == 'q')
            {
                std::cout << "The Following Vehicle has left the platoon " << std::endl;
                break;
            }
        }

        sleep(1);
    }

    restoreTerminalSettings(); // Restore the terminal settings

    follower.stopServer();

    return 0;
}