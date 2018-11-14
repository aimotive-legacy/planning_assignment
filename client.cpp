#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Rand.hh>

#include "custom_messages.pb.h"

#include <gazebo/gazebo_client.hh>

/**
 * Minimal client application to connect to the simulator, receive the
 * current world state and send the next ego vehicle velocity.
 * You need to extend the OnWorldStateReceived function, the rest
 * is the boilerplate code for communication.
 */

typedef const boost::shared_ptr<
        const custom_messages::WorldState>
        WorldStateRequestPtr;

typedef const boost::shared_ptr<
        const custom_messages::Statistics>
        StatisticsRequestPtr;

class Controller {
    private: gazebo::transport::NodePtr node;

    private: gazebo::transport::SubscriberPtr world_sub;
    private: gazebo::transport::SubscriberPtr statistics_sub;
    private: gazebo::transport::PublisherPtr pub;

    private: std::string worldTopicName = "~/world_state";
    private: std::string statisticsTopicName = "~/statistics";
    private: std::string commandTopicName = "~/client_command";

    private: int32_t simulation_round = 0;

    public: void Init()
    {
        // Create our node for communication
        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();

        // Subscribe to the topic, and register a callback
        this->world_sub = node->Subscribe(worldTopicName, &Controller::OnWorldStateReceived, this);

        this->statistics_sub = node->Subscribe(statisticsTopicName, &Controller::OnStatisticsReceived, this);

        // Publish to the velodyne topic
        this->pub = node->Advertise<custom_messages::Command>(commandTopicName);

        // Wait for a subscriber to connect to this publisher
        this->pub->WaitForConnection();
    }

    // Called when the simulator resets the world after reaching the goal, running out of time or crashing
    public: void ResetWorld()
    {
        std::cout << "New simulation round started, resetting world." << std::endl;
    }

    public: void PrintWorldStateMessage(WorldStateRequestPtr& msg) const
    {
        std::cout << "Simulation round: " << msg->simulation_round() << "; "
                  << "time: " << msg->time().sec() << "." << msg->time().nsec() << std::endl;
        std::cout << "  ego_car "
                  << " lane id: " << msg->ego_vehicle().lane_id()
                  << " p: (" << msg->ego_vehicle().position().x() << ", " << msg->ego_vehicle().position().y()
                  << ") v: (" << msg->ego_vehicle().velocity().x() << ", " << msg->ego_vehicle().velocity().y() << "); "
                  << std::endl;
        for (const auto& vehicle_msg : msg->vehicles())
        {
            std::cout << "  car id " << vehicle_msg.vehicle_id()
                      << " lane id: " << vehicle_msg.lane_id()
                      << " p: (" << vehicle_msg.position().x() << ", " << vehicle_msg.position().y()
                      << ") v: (" << vehicle_msg.velocity().x() << ", " << vehicle_msg.velocity().y() << "); "
                      << std::endl;
        }

        std::cout << std::endl;
    }

    public: void PrintStatisticsMessage(StatisticsRequestPtr& msg) const
    {
        std::cout << "Statistics from previous round: "
                  << "Success: " << msg->success() << ", "
                  << "collision: " << msg->collision_detected() << ", "
                  << "Time steps: "
                  << msg->simulation_time_steps_taken() << ", "
                  << "total acceleration: " << msg->total_acceleration() << ", "
                  << "acceleration/jerk limits respected: " << msg->limits_respected()
                  << std::endl;
    }

    // Called every time a new update is received from the simulator.
    // This is the main function where you should implement your code
    public: void OnWorldStateReceived(WorldStateRequestPtr& msg)
    {
        PrintWorldStateMessage(msg);

        // If the simulation round is different then this is a whole new setting, reinitialize the world
        if (msg->simulation_round() != simulation_round)
        {
            ResetWorld();
            simulation_round = msg->simulation_round();
        }

        // Calculate the next velocity for the ego car and send the response.
        // Implement your code here
        custom_messages::Command response_msg;
        response_msg.set_ego_car_speed(ignition::math::Rand::DblUniform(5, 15));
        response_msg.set_simulation_round(msg->simulation_round());
        this->pub->Publish(response_msg);
    }

    public: void OnStatisticsReceived(StatisticsRequestPtr& msg)
    {
        PrintStatisticsMessage(msg);
    }
};

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    Controller controller;
    controller.Init();

    // for (int i = 0; i < 100; ++i)
    while (true)
        gazebo::common::Time::MSleep(100);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
