#include "lead.h"

LeadingVehicle::LeadingVehicle(int id, double initialPosition, double initialSpeed)
    : id(id), position(initialPosition), speed(initialSpeed), serverSocket(0) 
    {
        // Initialize the clock matrix with zeros
        clockMatrix = std::vector<std::vector<int>>(2, std::vector<int>(2, 0));
    }

int LeadingVehicle::getId() const
{
    return id;
}

double LeadingVehicle::getPosition() const
{
    return position;
}

double LeadingVehicle::getSpeed() const
{
    return speed;
}

void LeadingVehicle::incrementClockMatrix(int value)
{
    // Increment each element in the clock matrix by the given value
    for (auto& row : clockMatrix)
    {
        for (auto& element : row)
        {
            element += value;
        }
    }
}

void LeadingVehicle::printClockMatrix()
{
    std::cout << "Clock Matrix:" << std::endl;
    for (const auto& row : clockMatrix)
    {
        for (const auto& element : row)
        {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

void LeadingVehicle::startServer()
{
    createServerSocket();

    std::cout << "Waiting for following vehicles to join..." << std::endl;

    std::thread followerThread(&LeadingVehicle::acceptFollowers, this);
    followerThread.detach();

    std::thread sendThread(&LeadingVehicle::sendMessageToFollowers, this);
    sendThread.detach();
}

void LeadingVehicle::stopServer()
{
    close(serverSocket);
}

void LeadingVehicle::createServerSocket()
{
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        std::cerr << "Error creating server socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(PORT);

    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "Error binding server socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(serverSocket, 1) < 0)
    {
        std::cerr << "Error listening on server socket" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void LeadingVehicle::acceptFollowers()
{
    while (true)
    {
        int clientSocket = accept(serverSocket, nullptr, nullptr);
        if (clientSocket < 0)
        {
            std::cerr << "Error accepting client connection" << std::endl;
            continue;
        }
        addFollower(clientSocket);
        std::thread followerThread(&LeadingVehicle::handleFollowerConnection, this, clientSocket);
        followerThread.detach();
    }
}

void LeadingVehicle::addFollower(int followerSocket)
{
    followers.push_back(followerSocket);
}

void LeadingVehicle::sendMessageToFollowers()
{
    while (true)
    {
        sendStateToFollowers();
        printState();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

void LeadingVehicle::sendStateToFollowers()
{
    Message message;
    message.senderId = id;
    message.position = position;
    message.speed = speed;
    // message.clockMatrix = clockMatrix;
    
    // Parallelize the loop using OpenMP
    #pragma omp parallel for
    for (int followerSocket : followers)
    {
        if (send(followerSocket, &message, sizeof(message), 0) < 0)
        {
            std::cerr << "Error sending state to follower" << std::endl;
        }
    }
}
void LeadingVehicle::handleFollowerConnection(int clientSocket)
{
    while (true)
    {
        Message followerMessage;
        int bytesRead = recv(clientSocket, &followerMessage, sizeof(followerMessage), 0);
        if (bytesRead <= 0)
        {
            auto it = std::find(followers.begin(), followers.end(), clientSocket);
            if (it != followers.end())
            {
                followers.erase(it);
                std::cout << "Follower ID " << followerMessage.senderId << " is disconnected" << std::endl;

                // Remove the disconnected vehicle Id from the vector
                auto it2 = std::remove(followerId.begin(), followerId.end(), followerMessage.senderId);
                followerId.erase(it2, followerId.end());
            }
            break;
        }

        if (std::find(followerId.begin(), followerId.end(), followerMessage.senderId) == followerId.end())
        {
            followerId.push_back(followerMessage.senderId);
        }

        followersPosition[followerMessage.senderId] = followerMessage.position;
        followersSpeed[followerMessage.senderId] = followerMessage.speed;
        
        _obstacleDetected = followerMessage.obstacleDetected;
        // Check if obstacle is detected by the follower
        if (_obstacleDetected)
        {
            std::cout << "Obstacle Detected by the follower" << std::endl;
            while(speed>0)
            {
                speed--; // Set leader's speed to 0 if obstacle is detected by the follower
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
    }
}

void LeadingVehicle::printState()
{
    std::cout << "Leading Vehicle ID: " << id;
    std::cout << "  Position: " << position;
    std::cout << "  Speed: " << speed << std::endl;
    
    #pragma omp parallel for
    for (int iD : followerId)
    {
        if (_obstacleDetected)
        {
            std::cout << "Follower ID " << iD << " is facing an obstacle" << std::endl;
        } 
        std::cout << "Connected Follower ID: " << iD;
        std::cout << "  Position: " << followersPosition[iD];
        std::cout << "  Speed: " << followersSpeed[iD] << std::endl;
    }
    std::cout << std::endl;
    
    printClockMatrix();
}

int main()
{
    LeadingVehicle leadingVehicle(1, 50, 60.0);
    leadingVehicle.startServer();

    while (true)
    {   
        #pragma omp parallel for
        // Increment the clock matrix by 1
        leadingVehicle.incrementClockMatrix(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Perform other tasks in the main function, if any
    }

    leadingVehicle.stopServer();
    return 0;
}