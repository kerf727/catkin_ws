#include "ros/ros.h"
// #include <iostream>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "gazebo_msgs/GetLinkState.h"
#include <queue>
#include <unordered_map>

class FollowPath
{
public:
    FollowPath()
    {
        this->node = node;
        using Graph = std::unordered_map<GraphPoint, std::vector<GraphPoint>>;

        ROS_INFO("Publishing Follow Path Node Data...");
        this->twistPublisher = node.advertise<geometry_msgs::Twist>("hexapod/teleop/twist", 1);
        this->buttonPublisher = node.advertise<std_msgs::Bool>("hexapod/teleop/button", 1);
        
        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
        node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);

        init_pos = getPosition();
        init_heading = getOrientation();
        position_error = 0.0;
        heading_error = 0.0;

        target_speed = 0.04;
        target_yaw = 0.15;

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        button_msg.data = false;

        publish_rate_Hz = 10;
        publish_rate_s = 0.1;
        // timer = node.createTimer(ros::Duration(publish_rate), boost::bind(&FollowPath::publishPathPeriodically, this));

        ROS_INFO("Publishing Path Commands.\n");
        publishPath();
    }

    ~FollowPath()
    {
        this->node.shutdown();
    }

private:

    ros::NodeHandle node;
    ros::Publisher twistPublisher;
    ros::Publisher buttonPublisher;
    ros::ServiceClient linkStateClient;
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool button_msg;
    double publish_rate_Hz, publish_rate_s;
    ros::Timer timer;
    boost::mutex publish_mutex;
    std::string gait_type = "tetrapod";
    double max_speed, max_yaw;
    double target_speed, target_yaw;
    double command_elapsed, command_start;
    geometry_msgs::Point curr_pos, init_pos;
    double curr_heading, init_heading;
    double distance_traveled;
    double position_error, heading_error;
    double position_eps = 1e-2;
    double heading_eps = 1e-2;

    void publishPath()
    {
        // Generate map of nodes to traverse
        Graph graph = generateGraph();
        ROS_INFO("Generated Graph.");
        
        // Set start and goal points from nodes in map
        // TODO: make sure these are both nodes in the graph
        GraphPoint start{0.0, 0.0};
        GraphPoint goal{-0.3, -0.3};
        ROS_INFO("Set start and goal nodes.");
        
        // Solve map using A star algorithm
        std::vector<GraphPoint> path = solveAStar(graph, start, goal);
        if (path.empty())
        {
            ROS_INFO("Failed to find a path.");
            return;
        }
        ROS_INFO("Solved path using A*.");
        int node_counter = 1;
        for (GraphPoint node : path)
        {
            ROS_INFO("Path node %d: (%.3f, %.3f)", node_counter, node.x, node.y);
            node_counter++;
        }

        // Traverse waypoints of solved path
        for (GraphPoint node : path)
        {
            ROS_INFO("\nNavigating to waypoint (%.3f, %.3f)", node.x, node.y);
            pathWaypoint(node.x, node.y, 0.0, target_speed, target_yaw);
        }
        ROS_INFO("Reached goal node.");
    }

    void pathWaypoint(
        const double& target_pos_x, const double& target_pos_y,
        const double& target_heading,
        const double& target_speed, const double& target_yaw
    )
    {
        ros::Rate rate(publish_rate_Hz);

        double error_x, error_y, max_error;
        double speed_ratio, yaw_ratio;
        double target_speed_, target_yaw_;

        target_speed_ = target_speed;
        target_yaw_ = target_yaw;
        if (target_speed_ > max_speed)
        {
            target_speed_ = max_speed;
        }
        if (target_yaw_ > max_yaw)
        {
            target_yaw_ = max_yaw;
        }
        if (target_yaw_ < -max_yaw)
        {
            target_yaw_ = -max_yaw;
        }
        speed_ratio = abs(target_speed/max_speed);
        yaw_ratio = abs(target_yaw/max_yaw);
        
        // Move to correct position
        position_error = sqrt(pow(target_pos_x - curr_pos.x, 2) +
                              pow(target_pos_y - curr_pos.y, 2));
        while (position_error > position_eps)
        {
            // Figure out what path command needs to be sent to get to waypoint
            curr_pos = getPosition();
            error_x = target_pos_x - curr_pos.x;
            error_y = target_pos_y - curr_pos.y;
            position_error = sqrt(pow(error_x, 2) +
                                  pow(error_y, 2));
            max_error = (abs(error_x) > abs(error_y)) ? abs(error_x) : abs(error_y);
            twist_msg.linear.x = speed_ratio*error_x/max_error;
            twist_msg.linear.y = speed_ratio*error_y/max_error;
            twist_msg.angular.x = 0.0;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
            ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
                curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);
            
            rate.sleep();
        }

        // Rotate to correct heading
        heading_error = abs(target_heading - curr_heading);
        while (heading_error > heading_eps)
        {
            // Figure out what path command needs to be sent to get to waypoint
            curr_heading = getOrientation();
            heading_error = abs(target_heading - curr_heading);
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = (target_heading > curr_heading) ? yaw_ratio : -yaw_ratio;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
            ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
                curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);

            rate.sleep();
        }

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.x = 0.0;
        twistPublisher.publish(twist_msg);

        ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
        ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
            curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);
    }

    void pathCommand(
        const double& linearX, const double& linearY,
        const double& angular, const double& runtime)
    {
        command_start = ros::Time::now().toSec();
        command_elapsed = 0.0;
        ros::Rate rate(publish_rate_Hz);
        while (command_elapsed <= runtime)
        {
            command_elapsed = ros::Time::now().toSec() - command_start;
            twist_msg.linear.x = linearX;
            twist_msg.linear.y = linearY;
            twist_msg.angular.x = angular;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);

            rate.sleep();
        }
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.x = 0.0;
        twistPublisher.publish(twist_msg);

        ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
    }

    geometry_msgs::Point getPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link"; // hexapod/base_link
        linkStateMsg.request.reference_frame = "ground::link"; // world
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    double getOrientation()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link";
        linkStateMsg.request.reference_frame = "ground::link";
        this->linkStateClient.call(linkStateMsg);
        // negative sign to match yaw convention
        return -linkStateMsg.response.link_state.pose.orientation.z;
    }

    struct GraphPoint
    {
        double x, y;

        bool operator==(const GraphPoint& other) const
        {
            return (x == other.x && y == other.y);
        }

        bool operator!=(const GraphPoint& other) const
        {
            return (x != other.x || y != other.y);
        }

        bool operator<(const GraphPoint& other) const
        {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };

    struct GraphPointHasher
    {
        std::size_t operator()(const GraphPoint& g) const
        {
            // Compute individual hash values for node.x and node.y
            // and combine them using XOR and bit shifting:
            return ((std::hash<double>()(g.x)
                    ^ (std::hash<double>()(g.y) << 1)) >> 1);
        }
    };
    
    struct Graph
    {
        std::unordered_map<GraphPoint, std::vector<GraphPoint>, GraphPointHasher> points;

        bool operator==(const Graph& other) const
        {
            return (points == other.points);
        }
    };
    
    template<typename T, typename priority_t>
    struct PriorityQueue
    {
        typedef std::pair<priority_t, T> PQElement;
        std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>> elements;

        inline bool empty() const
        {
            return elements.empty();
        }

        inline void put(T item, priority_t priority)
        {
            elements.emplace(priority, item);
        }

        T get()
        {
            T best_item = elements.top().second;
            elements.pop();
            return best_item;
        }

        auto clear()
        {
            while (!elements.empty())
            {
                elements.pop();
            }
            return elements;
        }
    };

    Graph generateGraph()
    {
        Graph graph;
        GraphPoint A{0.0, 0.0};
        GraphPoint B{0.0, 0.1};
        GraphPoint C{-0.2, -0.1};
        GraphPoint D{0.2, 0.3};
        GraphPoint E{0.1, -0.2};
        GraphPoint F{0.3, -0.1};
        GraphPoint G{-0.3, 0.2};
        GraphPoint H{-0.1, 0.3};
        GraphPoint I{-0.3, -0.3};
        GraphPoint J{0.3, 0.2};

        // Add points and their neighbors
        graph.points[A] = {B, E};
        graph.points[B] = {D};
        graph.points[C] = {A, E, G, H};
        graph.points[D] = {A, E, J};
        graph.points[E] = {C, D, F};
        graph.points[F] = {J};
        graph.points[G] = {I};
        graph.points[H] = {B, G};
        graph.points[I] = {C};
        graph.points[J] = {F};

        // for (auto node : graph.points)
        // {
        //     for (GraphPoint nbr : graph.points[node.first])
        //     {
        //         ROS_INFO("(%.3f, %.3f) neighbor: (%.3f, %.3f)", node.first.x, node.first.y, nbr.x, nbr.y);
        //     }
        // }

        return graph;
    }

    std::vector<GraphPoint> solveAStar(
        Graph& graph,
        const GraphPoint& start,
        const GraphPoint& goal)
    {
        PriorityQueue<GraphPoint, double> open_set;
        std::map<GraphPoint, GraphPoint> came_from;
        std::map<GraphPoint, double> cost_so_far; // TODO: make default value infinity?
        std::vector<GraphPoint> path;
         
        // Make sure these are empty to start
        open_set.clear();
        came_from.clear();
        cost_so_far.clear();
        path.clear();
        
        ROS_INFO("start: (%.3f, %.3f). goal: (%.3f, %.3f)", start.x, start.y, goal.x, goal.y);

        open_set.put(start, 0.0);
        came_from[start] = start;
        cost_so_far[start] = 0.0;

        while (!open_set.empty())
        {
            GraphPoint current = open_set.get();
            // ROS_INFO("current: (%.3f, %.3f)", current.x, current.y);
            if (current == goal)
            {
                while (current != start)
                {
                    path.insert(path.begin(), current);
                    current = came_from[current];
                }
                return path;
            }

            std::vector<GraphPoint> neighbors = graph.points[current];
            for (GraphPoint nbr : neighbors)
            {
                double new_cost = cost_so_far[current] + getCost(current, nbr);
                if (cost_so_far.find(nbr) == cost_so_far.end() || new_cost < cost_so_far[nbr])
                {
                    cost_so_far[nbr] = new_cost;
                    double priority = new_cost + heuristic(nbr, goal);
                    open_set.put(nbr, priority);
                    came_from[nbr] = current;
                }
            }
        }

        ROS_INFO("Failed. No path found.");
        return path; // failure
    }

    inline double heuristic(
        const GraphPoint& a,
        const GraphPoint& b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    inline double getCost(
        const GraphPoint& a,
        const GraphPoint& b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Follow Path Node...");
    ros::init(argc, argv, "follow_path");
    ROS_INFO("Initialized ros...");

    FollowPath follow_path;
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
