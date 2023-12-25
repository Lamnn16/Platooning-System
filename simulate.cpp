#include <iostream>
#include <vector>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

const int PORT = 8080;  // Port number for communication

// Define a message structure for communication between vehicles
struct Message {
    int senderId;        // ID of the sending vehicle
    double position;     // Position of the sending vehicle
    double speed;        // Speed of the sending vehicle
    // Add additional fields as needed for your communication protocol
};

class Vehicle {
private:
    int id;                   // Unique identifier for the vehicle
    bool isLead;              // Flag indicating if the vehicle is the lead vehicle
    double position;          // Current position of the vehicle
    double speed;             // Current speed of the vehicle
    double acceleration;      // Current acceleration of the vehicle
    bool isConnected;         // Flag indicating if the vehicle is currently connected
    int socket;               // Socket for communication

public:
    std::vector<int> followers;
    

    Vehicle(int id, bool isLead, double initialPosition, double initialSpeed)
        : id(id), isLead(isLead), position(initialPosition), speed(initialSpeed), acceleration(0.0), isConnected(false), socket(0) {}

    int getId() const {
        return id;
    }

    bool getIsLead() const {
        return isLead;
    }

    double getPosition() const {
        return position;
    }

    double getSpeed() const {
        return speed;
    }

    double getAcceleration() const {
        return acceleration;
    }

    void setPosition(double newPosition) {
        position = newPosition;
    }

    void setSpeed(double newSpeed) {
        speed = newSpeed;
    }

    void setAcceleration(double newAcceleration) {
        acceleration = newAcceleration;
    }

    void addFollower(int followerId) {
        followers.push_back(followerId);
    }

    bool isConnectedToPlatoon() const {
        return isConnected;
    }

    void connectToPlatoon(int socket) {
        isConnected = true;
        this->socket = socket;
    }

    void disconnectFromPlatoon() {
        isConnected = false;
        close(socket);
    }

    void sendMessage(const Message& message) const {
        if (isConnected)
            send(socket, &message, sizeof(message), 0);
    }
    
    void receiveMessage() const {
        char buffer[sizeof(Message)];
        ssize_t bytesRead = recv(socket, buffer, sizeof(buffer), 0);
        if (bytesRead > 0) {
            Message receivedMessage;
            memcpy(&receivedMessage, buffer, sizeof(Message));
            std::cout << "Received message at Vehicle " << id << ": "
                      << "Sender ID: " << receivedMessage.senderId
                      << ", Position: " << receivedMessage.position
                      << ", Speed: " << receivedMessage.speed
                      << std::endl;
        } else {
            // Handle error or connection closed
            std::cout << "Error receiving message at Vehicle " << id << std::endl;
        }
    }
};

void handleConnection(Vehicle& vehicle) {
    int serverSocket, clientSocket;
    sockaddr_in serverAddress{}, clientAddress{};
    int addrlen = sizeof(sockaddr);

    // Create server socket
    if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        std::cerr << "Failed to create socket\n";
        return;
    }

    // Set up server address
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(PORT + vehicle.getId());

    // Bind the server socket to the specified port
    if (bind(serverSocket, reinterpret_cast<sockaddr*>(&serverAddress), sizeof(serverAddress)) < 0) {
        std::cerr << "Failed to bind socket\n";
        return;
    }

    // Start listening for connections
    if (listen(serverSocket, 1) < 0) {
        std::cerr << "Failed to listen for connections\n";
        return;
    }

    // Accept a client connection
    if ((clientSocket = accept(serverSocket, reinterpret_cast<sockaddr*>(&clientAddress), reinterpret_cast<socklen_t*>(&addrlen))) < 0) {
        std::cerr << "Failed to accept connection\n";
        return;
    }

    // Connect to the platoon
    vehicle.connectToPlatoon(clientSocket);
}

std::vector<Vehicle> vehicles;
double desiredSpacing = 10.0;


void addFollower(std::vector<Vehicle>& vehicles, int leaderId, int followerId) {
    vehicles[leaderId].addFollower(followerId);
}

void setInitialSpeed(std::vector<Vehicle>& vehicles, double speed) {
    vehicles[0].setSpeed(speed);
}

void electLeadVehicle(std::vector<Vehicle>& vehicles, int vehicleId) {
    vehicles[0].addFollower(vehicleId);
    vehicles[vehicleId].disconnectFromPlatoon();
    vehicles[vehicleId].setSpeed(vehicles[0].getSpeed());
}

void setDesiredSpacing(std::vector<Vehicle>& vehicles, double spacing) {
    desiredSpacing = spacing;
}

void simulatePlatooningSystem() {
    const int NUM_VEHICLES = 5;  // Number of vehicles in the platoon

    // Create vehicles
    std::vector<Vehicle> vehicles;
    vehicles.reserve(NUM_VEHICLES);

   // Create lead vehicle with initial speed of 40 km/h
    Vehicle leadVehicle(0, true, 0.0, 40.0);
    vehicles.push_back(leadVehicle);
    
    
    // Create follower vehicles
    for (int i = 1; i < NUM_VEHICLES; ++i) {
        double initialPosition = i * desiredSpacing;
        double initialSpeed = 0.0;  // Set all follower vehicles to 0 speed initially
        Vehicle follower(i, false, initialPosition, initialSpeed);
        vehicles.push_back(follower);
        vehicles[0].addFollower(i);
    }
    
    // testing section
    addFollower(vehicles, 0, 5);         // Add vehicle 5 as a follower to vehicle 0
    setInitialSpeed(vehicles, 50.0);     // Set the initial speed of the lead vehicle to 50 km/h
    electLeadVehicle(vehicles, 2);       // Elect vehicle 2 as the new lead vehicle
    setDesiredSpacing(vehicles, 15.0);   // Set the desired spacing between vehicles to 15 meters

    // Start platooning
    std::vector<std::thread> threads;
    threads.reserve(NUM_VEHICLES);

    // Start connection threads for each vehicle
    for (int i = 0; i < NUM_VEHICLES; ++i) {
        threads.emplace_back(std::thread(handleConnection, std::ref(vehicles[i])));
    }

    // Simulate platooning system
    while (true) {
        // Update vehicle positions, speeds, and accelerations
        for (int i = 0; i < NUM_VEHICLES; ++i) {
             
                 // Retrieve the speed of the lead vehicle
                double leadSpeed = vehicles[0].getSpeed();
    
                // Calculate the desired position for the follower vehicle
                double precedingPosition = vehicles[i - 1].getPosition();
                double desiredPosition = precedingPosition + desiredSpacing;
    
                // Adjust the speed of the follower vehicle based on the lead speed
                double desiredSpeed = leadSpeed;
                vehicles[i].setSpeed(desiredSpeed);
    
                // Update the position of the follower vehicle
                vehicles[i].setPosition(desiredPosition);
            

            // Send vehicle state to connected followers
            Message message;
            message.senderId = vehicles[i].getId();
            message.position = vehicles[i].getPosition();
            message.speed = vehicles[i].getSpeed();

            for (int followerId : vehicles[i].followers) {
                vehicles[followerId].sendMessage(message);
            }
        }

        // Receive messages from the lead vehicle and update state of follower vehicles
        if (vehicles[0].isConnectedToPlatoon()) {
            vehicles[0].receiveMessage();
        }
        
        // Print vehicle positions
        for (int i = 0; i < NUM_VEHICLES; ++i) {
            std::cout << "Vehicle " << vehicles[i].getId() << ": Position = " << vehicles[i].getPosition()
                      << ", Speed = " << vehicles[i].getSpeed() << std::endl;
        }

        // Sleep for a short duration (e.g., 100 milliseconds) before the next iteration
        usleep(100000);
    }

    // Disconnect all vehicles from the platoon
    for (int i = 0; i < NUM_VEHICLES; ++i) {
        vehicles[i].disconnectFromPlatoon();
    }

    // Join all connection threads
    for (int i = 0; i < NUM_VEHICLES; ++i) {
        threads[i].join();
    }
}

int main() {
    simulatePlatooningSystem();


    return 0;
}