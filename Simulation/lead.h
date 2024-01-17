/**
 * @file lead.h
 *
 * @brief This is the header file for simulating the leading vehicle.
 *
 * This header file contains the class declaration for the LeadingVehicle class
 * and other related structures and functions necessary for simulating a leading vehicle.
 * The LeadingVehicle class represents the vehicle that leads a convoy of following vehicles.
 * It provides functionalities for starting and stopping a server, accepting and handling
 * connections from followers, and sending state information to the followers.
 *
 * @ingroup DPS - FHDO
 *
 * @author [Lam Nguyen Nhat]
 *
 */

#ifndef LEAD_H
#define LEAD_H

#include <iostream>
#include <vector>
#include <map>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <algorithm>

const int PORT = 8080;

struct Message
{
    int senderId;
    double position;
    double speed;
    bool isConnected;
    bool obstacleDetected;
};

class LeadingVehicle
{
private:
    int id;
    double position;
    double speed;
    std::vector<int> followers;
    int serverSocket;
    std::vector<int> followerId;
    std::map<int, int> followersPosition;
    std::map<int, int> followersSpeed;
    bool _obstacleDetected;

public:
    /**
     * @brief Constructor for the LeadingVehicle class.
     *
     * @param id The ID of the leading vehicle.
     * @param initialPosition The initial position of the leading vehicle.
     * @param initialSpeed The initial speed of the leading vehicle.
     */
    LeadingVehicle(int id, double initialPosition, double initialSpeed);
    
    /**
     * @brief Getter for the ID of the leading vehicle.
     *
     * @return The ID of the leading vehicle.
     */
    int getId() const;
    
    /**
     * @brief Getter for the position of the leading vehicle.
     *
     * @return The position of the leading vehicle.
     */
    double getPosition() const;
    
    /**
     * @brief Getter for the speed of the leading vehicle.
     *
     * @return The speed of the leading vehicle.
     */
    double getSpeed() const;
    
    /**
     * @brief Starts the server for the leading vehicle.
     */
    void startServer();
    
    /**
     * @brief Stops the server for the leading vehicle.
     */
    void stopServer();

private:
    void createServerSocket();
    void acceptFollowers();
    void addFollower(int followerSocket);
    void sendMessageToFollowers();
    void sendStateToFollowers();
    void handleFollowerConnection(int clientSocket);
    void printState();
};

#endif // LEAD_H