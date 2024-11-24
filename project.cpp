#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <cmath>
#include <algorithm>
#include <climits>

// Constants for visualization
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float NODE_RADIUS = 10.0f;
const float CLICK_RADIUS = 10.0f;
const float ANIMATION_SPEED = 0.5f;

struct Node {
    std::string name;
    sf::Vector2f position;

    Node(const std::string& n, float x, float y) : name(n), position(x, y) {}
};

struct Edge {
    int from;
    int to;
    int weight;

    Edge(int f, int t, int w) : from(f), to(t), weight(w) {}
};

class DeliveryRouteVisualizer {
private:
    sf::RenderWindow window;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    sf::Font font;

    struct Order {
        std::vector<int> waypoints;  // Points to visit in any order
        bool isComplete;
        
        Order() : isComplete(false) {}
    };

    std::vector<Order> orders;
    std::queue<Order> activeOrders;
    std::vector<int> currentPath;
    
    bool animating;
    sf::Clock animationClock;
    float animationProgress;
    int currentSegment;
    sf::CircleShape deliveryMarker;

    // Calculate shortest distance between any two nodes using Dijkstra
    std::vector<std::vector<int>> calculateDistanceMatrix() {
        int n = nodes.size();
        std::vector<std::vector<int>> dist(n, std::vector<int>(n, INT_MAX));
        
        // Calculate shortest path between all pairs of nodes
        for (int i = 0; i < n; i++) {
            auto [distances, _] = dijkstra(i);
            for (int j = 0; j < n; j++) {
                dist[i][j] = distances[j];
            }
        }
        return dist;
    }

    // Find optimal order of waypoints using nearest neighbor algorithm
    std::vector<int> findOptimalWaypointOrder(const std::vector<int>& waypoints) {
        if (waypoints.size() <= 2) return waypoints;

        auto distMatrix = calculateDistanceMatrix();
        std::vector<int> optimalOrder;
        std::vector<bool> visited(waypoints.size(), false);
        
        // Start with the first waypoint
        optimalOrder.push_back(waypoints[0]);
        visited[0] = true;
        
        // Find nearest unvisited waypoint
        while (optimalOrder.size() < waypoints.size()) {
            int currentNode = optimalOrder.back();
            int nearestDist = INT_MAX;
            int nearestIdx = -1;
            
            for (size_t i = 0; i < waypoints.size(); i++) {
                if (!visited[i]) {
                    int dist = distMatrix[currentNode][waypoints[i]];
                    if (dist < nearestDist) {
                        nearestDist = dist;
                        nearestIdx = i;
                    }
                }
            }
            
            if (nearestIdx != -1) {
                optimalOrder.push_back(waypoints[nearestIdx]);
                visited[nearestIdx] = true;
            }
        }
        
        return optimalOrder;
    }

    std::pair<std::vector<int>, std::vector<int>> dijkstra(int start) const {
        std::vector<int> distances(nodes.size(), INT_MAX);
        std::vector<int> previous(nodes.size(), -1);
        std::set<std::pair<int, int>> queue;

        distances[start] = 0;
        queue.insert({0, start});

        while (!queue.empty()) {
            int current = queue.begin()->second;
            queue.erase(queue.begin());

            for (const auto& edge : edges) {
                if (edge.from == current || edge.to == current) {
                    int neighbor = (edge.from == current) ? edge.to : edge.from;
                    int newDist = distances[current] + edge.weight;

                    if (newDist < distances[neighbor]) {
                        queue.erase({distances[neighbor], neighbor});
                        distances[neighbor] = newDist;
                        previous[neighbor] = current;
                        queue.insert({newDist, neighbor});
                    }
                }
            }
        }
        return {distances, previous};
    }

    std::vector<int> findMultiPointPath(const std::vector<int>& waypoints) {
        if (waypoints.size() < 2) return {};
        
        // First optimize the order of waypoints
        std::vector<int> optimizedWaypoints = findOptimalWaypointOrder(waypoints);
        std::vector<int> completePath;
        
        // For each consecutive pair of optimized waypoints
        for (size_t i = 0; i < optimizedWaypoints.size() - 1; i++) {
            auto [distances, previous] = dijkstra(optimizedWaypoints[i]);
            std::vector<int> segment = reconstructPath(optimizedWaypoints[i], optimizedWaypoints[i + 1], previous);
            
            if (segment.empty()) return {}; // No valid path found
            
            // Add segment to complete path (skip duplicate node)
            if (!completePath.empty()) segment.erase(segment.begin());
            completePath.insert(completePath.end(), segment.begin(), segment.end());
        }
        
        return completePath;
    }

    std::vector<int> reconstructPath(int start, int end, const std::vector<int>& previous) const {
        if (previous[end] == -1 && start != end) {
            return std::vector<int>{};
        }
        
        std::vector<int> path;
        for (int at = end; at != -1; at = previous[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void handleInput(const sf::Event& event) {
        if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                handleMouseClick(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
            } else if (event.mouseButton.button == sf::Mouse::Right) {
                // Finish current order
                if (!orders.empty() && !orders.back().isComplete) {
                    orders.back().isComplete = true;
                }
            }
        }
    }

    void handleMouseClick(const sf::Vector2f& mousePos) {
        for (size_t i = 0; i < nodes.size(); ++i) {
            float dx = nodes[i].position.x - mousePos.x;
            float dy = nodes[i].position.y - mousePos.y;
            if (std::sqrt(dx * dx + dy * dy) < CLICK_RADIUS) {
                if (orders.empty() || orders.back().isComplete) {
                    // Start new order
                    Order newOrder;
                    newOrder.waypoints.push_back(static_cast<int>(i));
                    orders.push_back(newOrder);
                } else {
                    // Add waypoint to current order
                    auto& currentOrder = orders.back();
                    if (!currentOrder.waypoints.empty() && 
                        currentOrder.waypoints.back() == static_cast<int>(i)) {
                        return; // Prevent duplicate consecutive waypoints
                    }
                    currentOrder.waypoints.push_back(static_cast<int>(i));
                }
                break;
            }
        }
    }

    void updateAnimation() {
        if (animating) {
            float elapsed = animationClock.getElapsedTime().asSeconds();
            animationProgress += elapsed * ANIMATION_SPEED;
            animationClock.restart();

            if (animationProgress >= 1.0f) {
                animationProgress = 0.0f;
                currentSegment++;
                if (currentSegment >= static_cast<int>(currentPath.size()) - 1) {
                    animating = false;
                    if (!activeOrders.empty()) {
                        activeOrders.pop();
                    }
                }
            }
        } else {
            processNextOrder();
        }
    }

    void processNextOrder() {
        if (!orders.empty() && orders.front().isComplete) {
            activeOrders.push(orders.front());
            orders.erase(orders.begin());
        }
        
        if (!animating && !activeOrders.empty()) {
            const Order& order = activeOrders.front();
            currentPath = findMultiPointPath(order.waypoints);

            if (!currentPath.empty()) {
                animating = true;
                animationProgress = 0.0f;
                currentSegment = 0;
                animationClock.restart();
            } else {
                std::cerr << "Error: No valid path found through waypoints!" << std::endl;
                activeOrders.pop();
            }
        }
    }

    void drawEdges() {
        for (const auto& edge : edges) {
            sf::Vertex line[] = {
                sf::Vertex(nodes[edge.from].position, sf::Color::Blue),
                sf::Vertex(nodes[edge.to].position, sf::Color::Blue)
            };

            if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1) {
                if ((edge.from == currentPath[currentSegment] && edge.to == currentPath[currentSegment + 1]) ||
                    (edge.to == currentPath[currentSegment] && edge.from == currentPath[currentSegment + 1])) {
                    line[0].color = sf::Color::Red;
                    line[1].color = sf::Color::Red;
                }
            }

            window.draw(line, 2, sf::Lines);

            // Draw edge weights
            sf::Text weightText;
            weightText.setFont(font);
            weightText.setString(std::to_string(edge.weight));
            weightText.setCharacterSize(14);
            weightText.setFillColor(sf::Color::White);
            
            sf::Vector2f midpoint = (nodes[edge.from].position + nodes[edge.to].position) / 2.f;
            weightText.setPosition(midpoint);
            
            sf::FloatRect textBounds = weightText.getLocalBounds();
            weightText.setOrigin(textBounds.width / 2, textBounds.height / 2);
            
            window.draw(weightText);
        }
    }

    void drawNodes() {
        for (size_t i = 0; i < nodes.size(); ++i) {
            sf::CircleShape circle(NODE_RADIUS);
            circle.setPosition(nodes[i].position - sf::Vector2f(NODE_RADIUS, NODE_RADIUS));
            circle.setFillColor(sf::Color::Green);

            // Color current order waypoints
            if (!orders.empty() && !orders.back().isComplete) {
                const auto& currentOrder = orders.back();
                size_t waypointIndex = 0;
                for (int waypoint : currentOrder.waypoints) {
                    if (static_cast<int>(i) == waypoint) {
                        if (waypointIndex == 0) {
                            circle.setFillColor(sf::Color::Yellow); // Start
                        } else if (waypointIndex == currentOrder.waypoints.size() - 1) {
                            circle.setFillColor(sf::Color::Magenta); // Current end
                        } else {
                            circle.setFillColor(sf::Color::Cyan); // Intermediate
                        }
                    }
                    waypointIndex++;
                }
            }
            
            // Color active order waypoints
            if (!activeOrders.empty()) {
                const auto& activeOrder = activeOrders.front();
                size_t waypointIndex = 0;
                for (int waypoint : activeOrder.waypoints) {
                    if (static_cast<int>(i) == waypoint) {
                        if (waypointIndex == 0) {
                            circle.setFillColor(sf::Color::Yellow);
                        } else if (waypointIndex == activeOrder.waypoints.size() - 1) {
                            circle.setFillColor(sf::Color::Magenta);
                        } else {
                            circle.setFillColor(sf::Color::Cyan);
                        }
                    }
                    waypointIndex++;
                }
            }

            window.draw(circle);

            // Draw node names
            sf::Text nodeText;
            nodeText.setFont(font);
            nodeText.setString(nodes[i].name);
            nodeText.setCharacterSize(16);
            nodeText.setFillColor(sf::Color::White);
            
            sf::FloatRect textBounds = nodeText.getLocalBounds();
            nodeText.setOrigin(textBounds.width / 2, textBounds.height + NODE_RADIUS);
            nodeText.setPosition(nodes[i].position);
            
            window.draw(nodeText);
        }
    }

    void drawDeliveryMarker() {
        if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1) {
            sf::Vector2f startPos = nodes[currentPath[currentSegment]].position;
            sf::Vector2f endPos = nodes[currentPath[currentSegment + 1]].position;
            sf::Vector2f currentPos = startPos + (endPos - startPos) * animationProgress;
            
            deliveryMarker.setPosition(currentPos - sf::Vector2f(deliveryMarker.getRadius(), deliveryMarker.getRadius()));
            window.draw(deliveryMarker);
        }
    }

public:
    DeliveryRouteVisualizer() :
        window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Optimized Multi-Waypoint Delivery Route Visualization"),
        animating(false),
        animationProgress(0.0f),
        currentSegment(0) {

        window.setFramerateLimit(60);

        if (!font.loadFromFile("arial.ttf")) {
            throw std::runtime_error("Error loading font. Ensure arial.ttf is in the same directory.");
        }

        deliveryMarker.setRadius(8);
        deliveryMarker.setFillColor(sf::Color::Red);

        // Initialize nodes
        nodes = {
            Node("House1", 100, 100),
            Node("House2", 200, 50),
            Node("House3", 300, 150),
            Node("House4", 400, 100),
            Node("House5", 500, 50),
            Node("Store1", 150, 300),
            Node("Store2", 350, 250),
            Node("Store3", 450, 350)
        };

        // Initialize edges with weights
        edges = {
            Edge(0, 1, 10),
            Edge(1, 2, 20),
            Edge(2, 3, 10),
            Edge(3, 4, 15),
            Edge(0, 5, 25),
            Edge(5, 6, 20),
            Edge(6, 7, 30),
            Edge(4, 7, 35),
            Edge(2, 6, 15),
            Edge(1, 5, 30),
            Edge(3, 6, 20)
        };
    }

    void run() {
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                } else {
                    handleInput(event);
                }
            }

            updateAnimation();

            window.clear(sf::Color::Black);
            drawEdges();
            drawNodes();
            drawDeliveryMarker();
            window.display();
        }
    }
};

int main() {
    try {
        DeliveryRouteVisualizer visualizer;
        visualizer.run();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}