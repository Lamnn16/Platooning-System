/**
 * @file follow.h
 *
 * @brief This is the header file for simulating the following vehicles.
 *
 * This header file contains the class declaration for the FollowingVehicle class
 * and other related structures and functions necessary for simulating a following vehicle.
 * The FollowingVehicle class represents a vehicle that follows a leading vehicle in a convoy.
 * It provides functionalities for connecting to the leader, sending and receiving messages,
 * and controlling its position and speed based on the received information.
 *
 * @ingroup DPS - FHDO
 *
 * @author [Lam Nguyen Nhat]
 *
 */

#ifndef FOLLOW_H
#define FOLLOW_H

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <thread>

const int STDIN = 0;

const int PORT = 8080; // Port number for communication

struct Message
{
    // Define the message structure for communication
    int senderId;
    double position;
    double speed;
    bool isConnected;
};

class FollowingVehicle
{
private:
    int id;
    double position;
    double speed;
    int serverSocket;
    double targetDistance; // Predefined distance between vehicles
    double Kp;             // Proportional gain of the PID controller
    double Ki;             // Integral gain of the PID controller
    double Kd;             // Derivative gain of the PID controller
    double integralError;  // Integral error for the PID controller
    double previousError;  // Previous error for the PID controller

public:
    /**
     * @brief Constructor for the FollowingVehicle class.
     *
     * @param id The ID of the following vehicle.
     * @param initialSpeed The initial speed of the following vehicle.
     * @param initialPosition The initial position of the following vehicle.
     * @param targetDistance The predefined distance between vehicles.
     * @param Kp The proportional gain of the PID controller.
     * @param Ki The integral gain of the PID controller.
     * @param Kd The derivative gain of the PID controller.
     */

    FollowingVehicle(int id, double initialSpeed, double initialPosition, double targetDistance, double Kp, double Ki, double Kd);

    /**
     * @brief Getter for the ID of the following vehicle.
     *
     * @return The ID of the following vehicle.
     */
    int getId() const;

    /**
     * @brief Getter for the position of the following vehicle.
     *
     * @return The position of the following vehicle.
     */
    double getPosition() const;

    /**
     * @brief Getter for the speed of the following vehicle.
     *
     * @return The speed of the following vehicle.
     */
    double getSpeed() const;

    /**
     * @brief Setter for the position of the following vehicle.
     *
     * @param newPosition The new position of the following vehicle.
     */
    void setPosition(double newPosition);

    /**
     * @brief Setter for the speed of the following vehicle.
     *
     * @param newSpeed The new speed of the following vehicle.
     */
    void setSpeed(double newSpeed);

    /**
     * @brief Connects the following vehicle to the leader vehicle.
     *
     * @param ipAddress The IP address of the leading vehicle.
     */
    void connectToLeader(const std::string &ipAddress);

    /**
     * @brief Sends a follower message to the leader vehicle.
     */
    void sendFollowerMessage();

    /**
     * @brief Sends follower messages continuously at a specified interval.
     *
     * @param interval The interval between sending follower messages.
     */
    void sendFollowerMessagesContinuously(int interval);

    /**
     * @brief Receives the state information from the leader vehicle.
     */
    void receiveStateFromLeader();

    /**
     * @brief Receives messages from the leader vehicle continuously.
     */
    void receiveMessageFromLeader();

    /**
     * @brief Prints the state information of the following vehicle.
     */
    void printState();

    /**
     * @brief Stops the server socket connection.
     */
    void stopServer();
};

/**
 * @brief Sets the terminal in non-canonical mode and disables input buffering.
 */
void setNonCanonicalMode();

/**
 * @brief Restores the terminal settings.
 */
void restoreTerminalSettings();

/**
 * @brief Checks if there is keyboard input available.
 *
 * @return True if there is keyboard input available, false otherwise.
 */
bool isKeyPressed();

/**
 * @brief Parses the command-line arguments.
 *
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @param id Reference to the ID of the following vehicle.
 * @param initialSpeed Reference to the initial speed of the following vehicle.
 * @param initialPosition Reference to the initial position of the following vehicle.
 * @param targetDistance Reference to the target distance between vehicles.
 * @param Kp Reference to the proportional gain of the PID controller.
 * @param Ki Reference to the integral gain of the PID controller.
 * @param Kd Reference to the derivative gain of the PID controller.
 */
void parseCommandLineArguments(int argc, char *argv[], int &id, double &initialSpeed, double &initialPosition, double &targetDistance, double &Kp, double &Ki, double &Kd);

#endif // FOLLOW_H