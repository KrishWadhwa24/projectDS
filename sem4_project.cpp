#include <SFML/Graphics.hpp>
#include <vector>
#include<bits/stdc++.h>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <bitset>
#include <iomanip>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <queue>

// Custom hash function for pair of int and int
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

// Forward declaration
class TSPSolver;
class NetworkFlowSolver;

// Menu to select between TSP and Network Flow
void showMenu() {
    std::cout << "==================================" << std::endl;
    std::cout << "Algorithm Visualization Tool" << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "1. Traveling Salesman Problem (TSP)" << std::endl;
    std::cout << "2. Network Flow (Edmonds-Karp)" << std::endl;
    std::cout << "Enter your choice (1-2): ";
}

class TSPSolver {
public:
    // City structure
    struct City {
        sf::Vector2f position;
        sf::CircleShape shape;
        sf::Text label;
        
        City(float x, float y, int id, const sf::Font& font) : position(x, y) {
            shape.setRadius(10.f);
            shape.setFillColor(sf::Color::Red);
            shape.setOrigin(sf::Vector2f(10.f, 10.f));
            shape.setPosition(position);
            
            label.setFont(font);
            label.setString(std::to_string(id));
            label.setCharacterSize(15);
            label.setFillColor(sf::Color::White);
            
            sf::FloatRect textRect = label.getLocalBounds();
            label.setOrigin(sf::Vector2f(textRect.width / 2.0f, textRect.height / 2.0f));
            label.setPosition(sf::Vector2f(x, y - 20.f));
        }
    };

    // Edge structure
    struct Edge {
        sf::Vertex line[2];
        sf::Text distanceText;
        float distance;
        
        Edge(const sf::Vector2f& start, const sf::Vector2f& end, float dist, const sf::Font& font) : distance(dist) {
            line[0].position = start;
            line[0].color = sf::Color(150, 150, 150);
            
            line[1].position = end;
            line[1].color = sf::Color(150, 150, 150);
            
            distanceText.setFont(font);
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << dist;
            distanceText.setString(ss.str());
            distanceText.setCharacterSize(12);
            distanceText.setFillColor(sf::Color(200, 200, 200)); // Light gray
            
            // Position the text in the middle of the line
            sf::Vector2f midPoint((start.x + end.x) / 2.0f, (start.y + end.y) / 2.0f);
            distanceText.setPosition(midPoint);
        }
        
        void setHighlight(bool highlight) {
            if (highlight) {
                line[0].color = sf::Color::Yellow;
                line[1].color = sf::Color::Yellow;
                distanceText.setFillColor(sf::Color::Yellow);
            } else {
                line[0].color = sf::Color(150, 150, 150);
                line[1].color = sf::Color(150, 150, 150);
                distanceText.setFillColor(sf::Color(200, 200, 200));
            }
        }
    };
    
    // Animation ball
    struct AnimationBall {
        sf::CircleShape shape;
        int currentCityIndex;
        int targetCityIndex;
        float progress;
        float speed;
        
        AnimationBall() : currentCityIndex(0), targetCityIndex(0), progress(0.f), speed(0.005f) {
            shape.setRadius(6.f);
            shape.setFillColor(sf::Color::Green);
            shape.setOrigin(sf::Vector2f(6.f, 6.f));
        }
        
        void setPosition(const sf::Vector2f& pos) {
            shape.setPosition(pos);
        }
        
        void reset(int startIndex) {
            currentCityIndex = startIndex;
            targetCityIndex = startIndex;
            progress = 0.f;
        }
        
        bool update(const std::vector<int>& path, const std::vector<City>& cities) {
            if (progress >= 1.f) {
                currentCityIndex = targetCityIndex;
                
                // Find the next target in the path
                for (size_t i = 0; i < path.size(); i++) {
                    if (path[i] == currentCityIndex) {
                        targetCityIndex = path[(i + 1) % path.size()];
                        break;
                    }
                }
                
                progress = 0.f;
            }
            
            const sf::Vector2f& currentPos = cities[currentCityIndex].position;
            const sf::Vector2f& targetPos = cities[targetCityIndex].position;
            
            sf::Vector2f newPos = currentPos + (targetPos - currentPos) * progress;
            shape.setPosition(newPos);
            
            progress += speed;
            return true;
        }
    };
    
    TSPSolver() : startCity(0), isAnimating(false), animationComplete(false), 
                  isSolving(false), statusMessage("Left-click to add cities, Right-click to start"),
                  algorithm(Algorithm::NEAREST_NEIGHBOR) {
        // Initialize
        if (!font.loadFromFile("arial.ttf")) {
            // Try alternate fonts if arial.ttf is not available
            if (!font.loadFromFile("/usr/share/fonts/TTF/DejaVuSans.ttf")) {
                if (!font.loadFromFile("/usr/share/fonts/truetype/freefont/FreeSans.ttf")) {
                    std::cout << "Error loading font. Text will not be displayed properly." << std::endl;
                }
            }
        }
        
        statusText.setFont(font);
        statusText.setCharacterSize(18);
        statusText.setFillColor(sf::Color::White);
        statusText.setPosition(sf::Vector2f(10.f, 10.f));
        statusText.setString(statusMessage);
        
        infoText.setFont(font);
        infoText.setCharacterSize(16);
        infoText.setFillColor(sf::Color::White);
        infoText.setPosition(sf::Vector2f(10.f, 40.f));
        
        algoText.setFont(font);
        algoText.setCharacterSize(16);
        algoText.setFillColor(sf::Color::Cyan);
        algoText.setPosition(sf::Vector2f(10.f, 540.f));
        updateAlgoText();
    }
    
    ~TSPSolver() {
        stopSolverThread();
    }
    
    void run() {
        sf::RenderWindow window(sf::VideoMode(800, 600), "Traveling Salesman Problem Visualization");
        window.setFramerateLimit(60);
        
        // Add some default cities
        addCity(100.f, 100.f);
        addCity(200.f, 300.f);
        addCity(400.f, 150.f);
        addCity(600.f, 350.f);
        addCity(300.f, 450.f);
        
        createAllEdges();
        updateInfoText();
        
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    stopSolverThread();
                    window.close();
                }
                else if (event.type == sf::Event::MouseButtonPressed) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    sf::Vector2f worldPos = window.mapPixelToCoords(mousePos);
                    
                    if (event.mouseButton.button == sf::Mouse::Left && !isAnimating && !isSolving) {
                        bool citySelected = false;
                        for (size_t i = 0; i < cities.size(); i++) {
                            sf::Vector2f cityPos = cities[i].position;
                            float distance = std::sqrt(
                                (worldPos.x - cityPos.x) * (worldPos.x - cityPos.x) +
                                (worldPos.y - cityPos.y) * (worldPos.y - cityPos.y)
                            );
                            
                            if (distance < 15.f) {
                                startCity = i;
                                statusMessage = "Start city set to " + std::to_string(startCity);
                                citySelected = true;
                                break;
                            }
                        }
                        
                        // If we didn't click on a city, add a new one
                        if (!citySelected) {
                            addCity(worldPos.x, worldPos.y);
                            createAllEdges();
                        }
                        updateInfoText();
                    }
                    else if (event.mouseButton.button == sf::Mouse::Right && cities.size() >= 3 && !isAnimating && !isSolving) {
                        // Start the algorithm and animation
                        startSolvingTSP();
                    }
                }
                else if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Key::R) {
                        // Reset
                        stopSolverThread();
                        cities.clear();
                        edges.clear();
                        optimalPath.clear();
                        isAnimating = false;
                        animationComplete = false;
                        isSolving = false;
                        startCity = 0;
                        statusMessage = "Left-click to add cities, Right-click to start";
                        updateInfoText();
                    }
                    else if (event.key.code == sf::Keyboard::Key::A) {
                        // Switch algorithm
                        if (!isSolving && !isAnimating) {
                            switchAlgorithm();
                        }
                    }
                    else if (event.key.code == sf::Keyboard::Key::Escape) {
                        stopSolverThread();
                        window.close();
                    }
                }
            }
            
            // Check if solver thread is done
            if (isSolving) {
                std::unique_lock<std::mutex> lock(solveMutex);
                if (solveFinished) {
                    isSolving = false;
                    startAnimation();
                    lock.unlock();
                }
            }
            
            // Update animation
            if (isAnimating && !cities.empty() && !optimalPath.empty()) {
                animationBall.update(optimalPath, cities);
            }
            
            // Clear and draw
            window.clear(sf::Color(50, 50, 50));
            
            // Draw edges
            for (const auto& edge : edges) {
                window.draw(edge.line, 2, sf::Lines);
                window.draw(edge.distanceText);
            }
            
            // Highlight optimal path edges if available
            if (!optimalPath.empty()) {
                for (size_t i = 0; i < optimalPath.size(); i++) {
                    int from = optimalPath[i];
                    int to = optimalPath[(i + 1) % optimalPath.size()];
                    
                    for (auto& edge : edges) {
                        if ((edge.line[0].position == cities[from].position && edge.line[1].position == cities[to].position) ||
                            (edge.line[0].position == cities[to].position && edge.line[1].position == cities[from].position)) {
                            edge.setHighlight(true);
                            window.draw(edge.line, 2, sf::Lines);
                            window.draw(edge.distanceText);
                        }
                    }
                }
            }
            
            // Draw cities
            for (const auto& city : cities) {
                window.draw(city.shape);
                window.draw(city.label);
            }
            
            // Draw the animation ball
            if (isAnimating && !cities.empty() && !optimalPath.empty()) {
                window.draw(animationBall.shape);
            }
            
            // Update and draw status text
            statusText.setString(statusMessage);
            window.draw(statusText);
            window.draw(infoText);
            window.draw(algoText);
            
            window.display();
        }
    }
    
private:
    enum class Algorithm {
        HELD_KARP,
        NEAREST_NEIGHBOR,
        GREEDY
    };

    std::vector<City> cities;
    std::vector<Edge> edges;
    std::vector<std::vector<float>> distances;
    std::vector<int> optimalPath;
    int startCity;
    sf::Font font;
    sf::Text statusText;
    sf::Text infoText;
    sf::Text algoText;
    std::string statusMessage;
    AnimationBall animationBall;
    bool isAnimating;
    bool animationComplete;
    bool isSolving;
    Algorithm algorithm;
    
    // Thread-related members
    std::thread solverThread;
    std::mutex solveMutex;
    std::condition_variable solveCV;
    bool solveFinished = false;
    
    void updateAlgoText() {
        std::string algoName;
        switch (algorithm) {
            case Algorithm::HELD_KARP:
                algoName = "Held-Karp (Exact, Dynamic Programming)";
                break;
            case Algorithm::NEAREST_NEIGHBOR:
                algoName = "Nearest Neighbor (Fast Heuristic)";
                break;
            case Algorithm::GREEDY:
                algoName = "Greedy (MST-based Heuristic)";
                break;
        }
        algoText.setString("Algorithm: " + algoName + " (Press A to switch)");
    }
    
    void switchAlgorithm() {
        switch (algorithm) {
            case Algorithm::HELD_KARP:
                algorithm = Algorithm::NEAREST_NEIGHBOR;
                break;
            case Algorithm::NEAREST_NEIGHBOR:
                algorithm = Algorithm::GREEDY;
                break;
            case Algorithm::GREEDY:
                algorithm = Algorithm::HELD_KARP;
                break;
        }
        updateAlgoText();
        statusMessage = "Algorithm switched. Right-click to solve.";
    }
    
    void addCity(float x, float y) {
        int id = cities.size();
        cities.emplace_back(x, y, id, font);
    }
    
    float calculateDistance(const sf::Vector2f& a, const sf::Vector2f& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    
    void createAllEdges() {
        edges.clear();
        
        // Create distance matrix
        int n = cities.size();
        distances.resize(n, std::vector<float>(n, 0.f));
        
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float dist = calculateDistance(cities[i].position, cities[j].position);
                distances[i][j] = dist;
                distances[j][i] = dist;
                
                edges.emplace_back(cities[i].position, cities[j].position, dist, font);
            }
        }
    }
    
    void updateInfoText() {
        std::stringstream ss;
        ss << "Cities: " << cities.size() << " | Start city: " << startCity;
        
        if (!optimalPath.empty()) {
            ss << " | Path length: " << std::fixed << std::setprecision(2) << calculatePathLength(optimalPath);
            ss << "\nPath: ";
            for (size_t i = 0; i < optimalPath.size(); i++) {
                ss << optimalPath[i];
                if (i < optimalPath.size() - 1) {
                    ss << " -> ";
                }
            }
            ss << " -> " << optimalPath[0];
        }
        
        ss << "\nControls: R - Reset, A - Change Algorithm, ESC - Exit";
        
        infoText.setString(ss.str());
    }
    
    float calculatePathLength(const std::vector<int>& path) {
        float length = 0.f;
        for (size_t i = 0; i < path.size(); i++) {
            int from = path[i];
            int to = path[(i + 1) % path.size()];
            length += distances[from][to];
        }
        return length;
    }
    
    void stopSolverThread() {
        if (solverThread.joinable()) {
            {
                std::unique_lock<std::mutex> lock(solveMutex);
                solveFinished = true;
            }
            solveCV.notify_one();
            solverThread.join();
        }
    }
    
    void startSolvingTSP() {
        if (isSolving) return;
        
        // Stop any existing solver thread
        stopSolverThread();
        
        // Reset state
        optimalPath.clear();
        isAnimating = false;
        animationComplete = false;
        solveFinished = false;
        isSolving = true;
        
        statusMessage = "Computing optimal path...";
        
        // Start the solver in a separate thread
        solverThread = std::thread([this]() {
            std::unique_lock<std::mutex> lock(solveMutex);
            
            switch (algorithm) {
                case Algorithm::HELD_KARP:
                    solveWithHeldKarp();
                    break;
                case Algorithm::NEAREST_NEIGHBOR:
                    solveWithNearestNeighbor();
                    break;
                case Algorithm::GREEDY:
                    solveWithGreedy();
                    break;
            }
            
            solveFinished = true;
            lock.unlock();
            solveCV.notify_one();
        });
    }
    
    void solveWithHeldKarp() {
        int n = cities.size();
        if (n <= 2) {
            if (n == 1) {
                optimalPath = {0};
            } else {
                optimalPath = {0, 1};
            }
            return;
        }
        
        // For large n, switch to nearest neighbor to avoid memory issues
        if (n > 15) {
            solveWithNearestNeighbor();
            return;
        }
        
        // Clear previous results
        optimalPath.clear();
        
        // Initialize memo table
        // key: pair<subset mask, current city>, value: pair<min distance, prev city>
        std::unordered_map<std::pair<int, int>, std::pair<float, int>, pair_hash> memo;
        
        // Start with visiting only the start city
        int allmask = (1 << n) - 1;
        
        // Function to find the shortest path
        std::function<std::pair<float, int>(int, int)> tsp = [&](int mask, int pos) -> std::pair<float, int> {
            // If all cities are visited
            if (mask == allmask) {
                return {distances[pos][startCity], pos};
            }
            
            // Check if we've already computed this state
            std::pair<int, int> key = {mask, pos};
            if (memo.find(key) != memo.end()) {
                return memo[key];
            }
            
            float ans = std::numeric_limits<float>::max();
            int nextCity = -1;
            
            // Try to go to each unvisited city
            for (int city = 0; city < n; city++) {
                if ((mask & (1 << city)) == 0) { // If city is not visited
                    auto [newDist, prev] = tsp(mask | (1 << city), city);
                    float totalDist = distances[pos][city] + newDist;
                    
                    if (totalDist < ans) {
                        ans = totalDist;
                        nextCity = city;
                    }
                }
            }
            
            memo[key] = {ans, nextCity};
            return memo[key];
        };
        
        // Start the TSP from the starting city
        int mask = (1 << startCity);
        auto [minDist, firstCity] = tsp(mask, startCity);
        
        // Reconstruct the path
        optimalPath.push_back(startCity);
        int currentCity = startCity;
        mask = (1 << startCity);
        
        while (mask != allmask) {
            std::pair<int, int> key = {mask, currentCity};
            int nextCity = memo[key].second;
            
            optimalPath.push_back(nextCity);
            mask |= (1 << nextCity);
            currentCity = nextCity;
        }
        
        updateInfoText();
    }
    
    void solveWithNearestNeighbor() {
        int n = cities.size();
        if (n <= 1) {
            optimalPath = {0};
            return;
        }
        
        // Clear previous results
        optimalPath.clear();
        
        // Start with the specified starting city
        optimalPath.push_back(startCity);
        std::vector<bool> visited(n, false);
        visited[startCity] = true;
        
        int current = startCity;
        
        // Add remaining cities
        for (int i = 1; i < n; i++) {
            int nextCity = -1;
            float minDistance = std::numeric_limits<float>::max();
            
            for (int j = 0; j < n; j++) {
                if (!visited[j] && distances[current][j] < minDistance) {
                    minDistance = distances[current][j];
                    nextCity = j;
                }
            }
            
            if (nextCity != -1) {
                optimalPath.push_back(nextCity);
                visited[nextCity] = true;
                current = nextCity;
            }
        }
        
        updateInfoText();
    }
    
    void solveWithGreedy() {
        int n = cities.size();
        if (n <= 1) {
            optimalPath = {0};
            return;
        }
        
        // Clear previous results
        optimalPath.clear();
        
        // Create a list of all possible edges
        std::vector<std::tuple<float, int, int>> allEdges;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                allEdges.push_back({distances[i][j], i, j});
            }
        }
        
        // Sort edges by distance
        std::sort(allEdges.begin(), allEdges.end());
        
        // Keep track of the degree of each vertex
        std::vector<int> degree(n, 0);
        
        // Union-Find data structure
        std::vector<int> parent(n);
        for (int i = 0; i < n; i++) {
            parent[i] = i;
        }
        
        std::function<int(int)> find = [&](int x) {
            if (parent[x] != x) {
                parent[x] = find(parent[x]);
            }
            return parent[x];
        };
        
        std::function<void(int, int)> unite = [&](int x, int y) {
            parent[find(x)] = find(y);
        };
        
        // Build MST with limited degree
        std::vector<std::pair<int, int>> mstEdges;
        for (const auto& [dist, u, v] : allEdges) {
            if (degree[u] < 2 && degree[v] < 2 && find(u) != find(v)) {
                unite(u, v);
                mstEdges.push_back({u, v});
                degree[u]++;
                degree[v]++;
            }
        }
        
        // Build adjacency list from MST
        std::vector<std::vector<int>> adj(n);
        for (const auto& [u, v] : mstEdges) {
            adj[u].push_back(v);
            adj[v].push_back(u);
        }
        
        // Find Eulerian path (or approximate it)
        std::vector<int> path;
        std::function<void(int)> dfs = [&](int node) {
            while (!adj[node].empty()) {
                int next = adj[node].back();
                adj[node].pop_back();
                
                // Remove the reverse edge
                adj[next].erase(std::remove(adj[next].begin(), adj[next].end(), node), adj[next].end());
                
                dfs(next);
            }
            path.push_back(node);
        };
        
        // Start DFS from the specified starting city
        dfs(startCity);
        
        // Remove duplicates to get a Hamiltonian path (shortcutting)
        std::vector<bool> visited(n, false);
        for (int i = 0; i < path.size(); i++) {
            if (!visited[path[i]]) {
                optimalPath.push_back(path[i]);
                visited[path[i]] = true;
            }
        }
        
        // If we missed any cities, add them using nearest insertion
        for (int i = 0; i < n; i++) {
            if (!visited[i]) {
                int bestPos = 0;
                float bestIncrease = std::numeric_limits<float>::max();
                
                for (int j = 0; j < optimalPath.size(); j++) {
                    int prev = optimalPath[j];
                    int next = optimalPath[(j + 1) % optimalPath.size()];
                    float increase = distances[prev][i] + distances[i][next] - distances[prev][next];
                    
                    if (increase < bestIncrease) {
                        bestIncrease = increase;
                        bestPos = j;
                    }
                }
                
                optimalPath.insert(optimalPath.begin() + bestPos + 1, i);
            }
        }
        
        // Rotate the path so that it starts with the startCity
        int startPos = std::find(optimalPath.begin(), optimalPath.end(), startCity) - optimalPath.begin();
        std::rotate(optimalPath.begin(), optimalPath.begin() + startPos, optimalPath.end());
        
        updateInfoText();
    }
    
    void startAnimation() {
        if (optimalPath.empty() || cities.empty()) return;
        
        // Reset the animation ball
        animationBall.reset(startCity);
        animationBall.setPosition(cities[startCity].position);
        
        isAnimating = true;
        animationComplete = false;
        statusMessage = "Animating optimal path...";
    }
};

class NetworkFlowSolver {
public:
    // Node structure
    struct Node {
        sf::Vector2f position;
        sf::CircleShape shape;
        sf::Text label;
        bool isSource;
        bool isSink;
        
        Node(float x, float y, int id, const sf::Font& font, bool source = false, bool sink = false) 
            : position(x, y), isSource(source), isSink(sink) {
            shape.setRadius(20.f);
            
            if (isSource) {
                shape.setFillColor(sf::Color::Green);
            } else if (isSink) {
                shape.setFillColor(sf::Color::Red);
            } else {
                shape.setFillColor(sf::Color::Blue);
            }
            
            shape.setOrigin(sf::Vector2f(20.f, 20.f));
            shape.setPosition(position);
            
            label.setFont(font);
            label.setString(std::to_string(id));
            label.setCharacterSize(18);
            label.setFillColor(sf::Color::White);
            
            sf::FloatRect textRect = label.getLocalBounds();
            label.setOrigin(sf::Vector2f(textRect.width / 2.0f, textRect.height / 2.0f));
            label.setPosition(position);
        }
    };

    // Flow edge structure
    struct FlowEdge {
        sf::Vertex line[2];
        sf::Text capacityText;
        sf::Text flowText;
        int from;
        int to;
        int capacity;
        int flow;
        bool isResidual;
        bool highlighted;
        
        FlowEdge(const sf::Vector2f& start, const sf::Vector2f& end, int from_, int to_, int cap, const sf::Font& font, bool residual = false) 
            : from(from_), to(to_), capacity(cap), flow(0), isResidual(residual), highlighted(false) {
                
            line[0].position = start;
            line[0].color = isResidual ? sf::Color(100, 100, 100, 100) : sf::Color(100, 100, 255);
            
            line[1].position = end;
            line[1].color = isResidual ? sf::Color(100, 100, 100, 100) : sf::Color(100, 100, 255);
            
            // Add an arrow to indicate direction
            sf::Vector2f dir = end - start;
            float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            sf::Vector2f unitDir = dir / length;
            sf::Vector2f normal(-unitDir.y, unitDir.x);
            
            // Position for the capacity text (centered on the edge)
            sf::Vector2f midPoint = start + dir * 0.5f;
            
            capacityText.setFont(font);
            capacityText.setString(std::to_string(capacity));
            capacityText.setCharacterSize(16);
            capacityText.setFillColor(sf::Color::White);
            
            // Position the capacity text above the edge
            sf::FloatRect textRect = capacityText.getLocalBounds();
            capacityText.setOrigin(sf::Vector2f(textRect.width / 2.0f, textRect.height / 2.0f));
            capacityText.setPosition(midPoint + normal * 15.f);
            
            // Create flow text (initially 0)
            flowText.setFont(font);
            flowText.setString("0");
            flowText.setCharacterSize(16);
            flowText.setFillColor(sf::Color(150, 255, 150)); // Light green
            
            // Position the flow text below the edge
            sf::FloatRect flowRect = flowText.getLocalBounds();
            flowText.setOrigin(sf::Vector2f(flowRect.width / 2.0f, flowRect.height / 2.0f));
            flowText.setPosition(midPoint - normal * 15.f);
        }
        
        void updateFlow(int newFlow) {
            flow = newFlow;
            flowText.setString(std::to_string(flow));
            
            // Update the color based on flow/capacity ratio
            float ratio = static_cast<float>(flow) / capacity;
            sf::Color edgeColor;
            
            if (isResidual) {
                edgeColor = sf::Color(100, 100, 100, 100); // Gray for residual
            } else if (ratio > 0.8f) {
                edgeColor = sf::Color(255, 100, 100); // Red for near capacity
            } else if (ratio > 0.4f) {
                edgeColor = sf::Color(255, 255, 100); // Yellow for medium flow
            } else if (ratio > 0.0f) {
                edgeColor = sf::Color(100, 255, 100); // Green for low flow
            } else {
                edgeColor = sf::Color(100, 100, 255); // Blue for no flow
            }
            
            if (highlighted) {
                edgeColor = sf::Color::Yellow;
            }
            
            line[0].color = edgeColor;
            line[1].color = edgeColor;
        }
        
        void setHighlight(bool highlight) {
            highlighted = highlight;
            if (highlight) {
                line[0].color = sf::Color::Yellow;
                line[1].color = sf::Color::Yellow;
                capacityText.setFillColor(sf::Color::Yellow);
                flowText.setFillColor(sf::Color::Yellow);
            } else {
                // Reset to normal color based on flow
                updateFlow(flow);
                capacityText.setFillColor(sf::Color::White);
                flowText.setFillColor(sf::Color(150, 255, 150));
            }
        }
    };
    
    // Animation ball
    struct FlowBall {
        sf::CircleShape shape;
        int currentNodeIndex;
        int targetNodeIndex;
        float progress;
        float speed;
        
        FlowBall() : currentNodeIndex(0), targetNodeIndex(0), progress(0.f), speed(0.005f) {
            shape.setRadius(8.f);
            shape.setFillColor(sf::Color(50, 255, 50, 200)); // Semi-transparent green
            shape.setOrigin(sf::Vector2f(8.f, 8.f));
        }
        
        void setPosition(const sf::Vector2f& pos) {
            shape.setPosition(pos);
        }
        
        void reset(int startIndex) {
            currentNodeIndex = startIndex;
            targetNodeIndex = startIndex;
            progress = 0.f;
        }
        
        bool update(const std::vector<int>& path, const std::vector<Node>& nodes) {
            if (progress >= 1.f) {
                currentNodeIndex = targetNodeIndex;
                
                // Find the position of currentNodeIndex in the path
                auto it = std::find(path.begin(), path.end(), currentNodeIndex);
                if (it != path.end() && std::next(it) != path.end()) {
                    targetNodeIndex = *std::next(it);
                } else {
                    // End of path, stop animation
                    return false;
                }
                
                progress = 0.f;
            }
            
            const sf::Vector2f& currentPos = nodes[currentNodeIndex].position;
            const sf::Vector2f& targetPos = nodes[targetNodeIndex].position;
            
            sf::Vector2f newPos = currentPos + (targetPos - currentPos) * progress;
            shape.setPosition(newPos);
            
            progress += speed;
            return true;
        }
    };
    
    NetworkFlowSolver() : sourceNode(0), sinkNode(1), isAnimating(false), 
                          isSolving(false), statusMessage("Left-click to add nodes, Right-click to find max flow") {
        // Initialize
        if (!font.loadFromFile("arial.ttf")) {
            // Try alternate fonts if arial.ttf is not available
            if (!font.loadFromFile("/usr/share/fonts/TTF/DejaVuSans.ttf")) {
                if (!font.loadFromFile("/usr/share/fonts/truetype/freefont/FreeSans.ttf")) {
                    std::cout << "Error loading font. Text will not be displayed properly." << std::endl;
                }
            }
        }
        
        statusText.setFont(font);
        statusText.setCharacterSize(18);
        statusText.setFillColor(sf::Color::White);
        statusText.setPosition(sf::Vector2f(10.f, 10.f));
        statusText.setString(statusMessage);
        
        infoText.setFont(font);
        infoText.setCharacterSize(16);
        infoText.setFillColor(sf::Color::White);
        infoText.setPosition(sf::Vector2f(10.f, 40.f));
        
        // Set up the default graph with source and sink
        addNode(100.f, 300.f, true, false);  // Source node
        addNode(700.f, 300.f, false, true);  // Sink node
        
        // Add some intermediate nodes
        addNode(300.f, 150.f);
        addNode(300.f, 450.f);
        addNode(500.f, 150.f);
        addNode(500.f, 450.f);
        
        // Add some edges
        addEdge(0, 2, 10);
        addEdge(0, 3, 8);
        addEdge(2, 4, 5);
        addEdge(3, 4, 10);
        addEdge(3, 5, 7);
        addEdge(4, 1, 7);
        addEdge(5, 1, 10);
        addEdge(2, 5, 3);
        
        updateInfoText();
    }
    
    ~NetworkFlowSolver() {
        stopSolverThread();
    }
    
    void run() {
        sf::RenderWindow window(sf::VideoMode(800, 600), "Network Flow Visualization (Edmonds-Karp)");
        window.setFramerateLimit(60);
        
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    stopSolverThread();
                    window.close();
                }
                else if (event.type == sf::Event::MouseButtonPressed) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    sf::Vector2f worldPos = window.mapPixelToCoords(mousePos);
                    
                    if (event.mouseButton.button == sf::Mouse::Left && !isAnimating && !isSolving) {
                        bool nodeSelected = false;
                        
                        // Check if we clicked on a node
                        for (size_t i = 0; i < nodes.size(); i++) {
                            sf::Vector2f nodePos = nodes[i].position;
                            float distance = std::sqrt(
                                (worldPos.x - nodePos.x) * (worldPos.x - nodePos.x) +
                                (worldPos.y - nodePos.y) * (worldPos.y - nodePos.y)
                            );
                            
                            if (distance < 20.f) {
                                // If we're in edge creation mode, create an edge
                                if (selectedNode != -1 && selectedNode != i) {
                                    // Show capacity input dialog
                                    int capacity = 10; // Default capacity
                                    std::cout << "Enter capacity for edge " << selectedNode << " -> " << i << ": ";
                                    std::cin >> capacity;
                                    
                                    addEdge(selectedNode, i, capacity);
                                    selectedNode = -1;
                                    statusMessage = "Edge added. Left-click to select a node.";
                                } else {
                                    selectedNode = i;
                                    statusMessage = "Node " + std::to_string(i) + " selected. Left-click another node to create an edge.";
                                }
                                nodeSelected = true;
                                break;
                            }
                        }
                        
                        // If we didn't click on a node and not in edge creation mode, add a new node
                        if (!nodeSelected && selectedNode == -1) {
                            addNode(worldPos.x, worldPos.y);
                            statusMessage = "Node added. Left-click to select a node.";
                        }
                        
                        updateInfoText();
                    }
                    else if (event.mouseButton.button == sf::Mouse::Right && nodes.size() >= 2 && !isAnimating && !isSolving) {
                        // Reset selection
                        selectedNode = -1;
                        
                        // Start the algorithm and animation
                        startSolvingFlow();
                    }
                }
                else if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Key::R) {
                        // Reset
                        stopSolverThread();
                        resetGraph();
                        updateInfoText();
                    }
                    else if (event.key.code == sf::Keyboard::Key::Escape) {
                        stopSolverThread();
                        window.close();
                    }
                }
            }
            
            // Check if solver thread is done
            if (isSolving) {
                std::unique_lock<std::mutex> lock(solveMutex);
                if (solveFinished) {
                    isSolving = false;
                    startAnimation();
                    lock.unlock();
                }
            }
            
            // Update animation
            if (isAnimating && currentAnimationPath.size() > 1) {
                if (!flowBall.update(currentAnimationPath, nodes)) {
                    // Animation finished for this path
                    if (!animationPaths.empty()) {
                        // Move to next path
                        currentAnimationPath = animationPaths.front();
                        animationPaths.pop();
                        flowBall.reset(currentAnimationPath[0]);
                        
                        // Update the flow along this path
                        int pathFlow = pathFlows.front();
                        pathFlows.pop();
                        
                        // Update the flow on the edges in this path
                        for (size_t i = 0; i < currentAnimationPath.size() - 1; i++) {
                            int u = currentAnimationPath[i];
                            int v = currentAnimationPath[i + 1];
                            
                            // Find the edge and update its flow
                            for (auto& edge : edges) {
                                if (edge.from == u && edge.to == v) {
                                    edge.updateFlow(edge.flow + pathFlow);
                                    break;
                                }
                            }
                        }
                    } else {
                        // All animations finished
                        isAnimating = false;
                        statusMessage = "Max flow found: " + std::to_string(maxFlow);
                    }
                }
            }
            
            // Clear and draw
            window.clear(sf::Color(50, 50, 50));
            
            // Draw edges
            for (const auto& edge : edges) {
                if (!edge.isResidual) {
                    window.draw(edge.line, 2, sf::Lines);
                    window.draw(edge.capacityText);
                    window.draw(edge.flowText);
                }
            }
            
            // Draw nodes
            for (const auto& node : nodes) {
                window.draw(node.shape);
                window.draw(node.label);
            }
            
            // Draw the animation ball
            if (isAnimating && currentAnimationPath.size() > 1) {
                window.draw(flowBall.shape);
            }
            
            // Update and draw status text
            statusText.setString(statusMessage);
            window.draw(statusText);
            window.draw(infoText);
            
            window.display();
        }
    }
    
private:
    std::vector<Node> nodes;
    std::vector<FlowEdge> edges;
    std::vector<std::vector<int>> adjacencyList;
    int sourceNode;
    int sinkNode;
    sf::Font font;
    sf::Text statusText;
    sf::Text infoText;
    std::string statusMessage;
    FlowBall flowBall;
    bool isAnimating;
    bool isSolving;
    int selectedNode = -1;
    int maxFlow = 0;
    
    // Animation paths
    std::queue<std::vector<int>> animationPaths;
    std::queue<int> pathFlows;
    std::vector<int> currentAnimationPath;
    
    // Thread-related members
    std::thread solverThread;
    std::mutex solveMutex;
    std::condition_variable solveCV;
    bool solveFinished = false;
    
    void resetGraph() {
        // Keep only source and sink
        nodes.clear();
        edges.clear();
        adjacencyList.clear();
        
        // Add source and sink
        addNode(100.f, 300.f, true, false);  // Source node
        addNode(700.f, 300.f, false, true);  // Sink node
        
        isAnimating = false;
        isSolving = false;
        selectedNode = -1;
        maxFlow = 0;
        
        // Clear animation queues
        std::queue<std::vector<int>> emptyPathQueue;
        std::queue<int> emptyFlowQueue;
        animationPaths.swap(emptyPathQueue);
        pathFlows.swap(emptyFlowQueue);
        currentAnimationPath.clear();
        
        statusMessage = "Left-click to add nodes, Right-click to find max flow";
    }
    
    void addNode(float x, float y, bool isSource = false, bool isSink = false) {
        int id = nodes.size();
        nodes.emplace_back(x, y, id, font, isSource, isSink);
        
        // Update adjacency list
        if (adjacencyList.size() <= id) {
            adjacencyList.resize(id + 1);
        }
    }
    
    void addEdge(int from, int to, int capacity) {
        if (from < 0 || from >= nodes.size() || to < 0 || to >= nodes.size()) return;
        
        // Check if edge already exists
        for (const auto& edge : edges) {
            if (edge.from == from && edge.to == to) {
                // Edge already exists
                return;
            }
        }
        
        // Add forward edge
        edges.emplace_back(nodes[from].position, nodes[to].position, from, to, capacity, font, false);
        
        // Add residual edge (initially with 0 capacity)
        edges.emplace_back(nodes[to].position, nodes[from].position, to, from, 0, font, true);
        
        // Update adjacency list for BFS
        adjacencyList[from].push_back(edges.size() - 2); // Index of forward edge
        adjacencyList[to].push_back(edges.size() - 1);   // Index of residual edge
    }
    
    void updateInfoText() {
        std::stringstream ss;
        ss << "Nodes: " << nodes.size() << " | Edges: " << edges.size() / 2;
        ss << " | Source: " << sourceNode << " | Sink: " << sinkNode;
        
        if (maxFlow > 0) {
            ss << " | Max Flow: " << maxFlow;
        }
        
        ss << "\nControls: R - Reset, ESC - Exit";
        
        infoText.setString(ss.str());
    }
    
    void stopSolverThread() {
        if (solverThread.joinable()) {
            {
                std::unique_lock<std::mutex> lock(solveMutex);
                solveFinished = true;
            }
            solveCV.notify_one();
            solverThread.join();
        }
    }
    
    void startSolvingFlow() {
        if (isSolving) return;
        stopSolverThread();
        
        // Reset state
        maxFlow = 0;
        isAnimating = false;
        solveFinished = false;
        isSolving = true;
        
        // Clear animation queues
        std::queue<std::vector<int>> emptyPathQueue;
        std::queue<int> emptyFlowQueue;
        animationPaths.swap(emptyPathQueue);
        pathFlows.swap(emptyFlowQueue);
        currentAnimationPath.clear();
        
        statusMessage = "Computing max flow...";
        
        // Reset all flows to 0
        for (auto& edge : edges) {
            edge.updateFlow(0);
        }
        
        // Start the solver in a separate thread
        solverThread = std::thread([this]() {
            std::unique_lock<std::mutex> lock(solveMutex);
            
            // Run Edmonds-Karp algorithm
            solveWithEdmondsKarp();
            
            solveFinished = true;
            lock.unlock();
            solveCV.notify_one();
        });
    }
    
    void solveWithEdmondsKarp() {
        maxFlow = 0;
        
        // Create residual graph capacity (edges already have forward and residual edges)
        int n = nodes.size();
        
        // Helper function to find an augmenting path using BFS
        auto bfs = [&]() -> std::vector<int> {
            std::vector<int> parent(n, -1);
            std::queue<int> q;
            
            q.push(sourceNode);
            parent[sourceNode] = -2; // Special value to indicate source
            
            while (!q.empty() && parent[sinkNode] == -1) {
                int u = q.front();
                q.pop();
                
                // Check all edges from u
                for (int edgeIdx : adjacencyList[u]) {
                    const auto& edge = edges[edgeIdx];
                    int v = edge.to;
                    
                    // If there's available capacity and v is not visited
                    if (edge.capacity > edge.flow && parent[v] == -1) {
                        parent[v] = edgeIdx;
                        q.push(v);
                    }
                }
            }
            
            // If we didn't reach sink, no augmenting path exists
            if (parent[sinkNode] == -1) {
                return {};
            }
            
            // Reconstruct the path
            std::vector<int> path;
            int current = sinkNode;
            while (current != sourceNode) {
                int edgeIdx = parent[current];
                int prev = edges[edgeIdx].from;
                path.push_back(current);
                current = prev;
            }
            path.push_back(sourceNode);
            
            // Reverse to get path from source to sink
            std::reverse(path.begin(), path.end());
            
            return path;
        };
        
        // Find augmenting paths
        while (true) {
            std::vector<int> path = bfs();
            if (path.empty()) break;
            
            // Find maximum flow on this path
            int pathFlow = std::numeric_limits<int>::max();
            for (size_t i = 0; i < path.size() - 1; i++) {
                int u = path[i];
                int v = path[i + 1];
                
                // Find the edge from u to v
                int capacity = 0;
                for (const auto& edge : edges) {
                    if (edge.from == u && edge.to == v) {
                        capacity = edge.capacity - edge.flow;
                        break;
                    }
                }
                
                pathFlow = std::min(pathFlow, capacity);
            }
            
            // Add path for animation
            animationPaths.push(path);
            pathFlows.push(pathFlow);
            
            // Update flows in the residual graph
            for (size_t i = 0; i < path.size() - 1; i++) {
                int u = path[i];
                int v = path[i + 1];
                
                // Find and update forward edge
                for (size_t j = 0; j < edges.size(); j++) {
                    if (edges[j].from == u && edges[j].to == v) {
                        edges[j].flow += pathFlow;
                        
                        // Find and update the residual edge
                        for (size_t k = 0; k < edges.size(); k++) {
                            if (edges[k].from == v && edges[k].to == u) {
                                edges[k].capacity = edges[j].flow;
                                edges[k].flow = 0;
                                break;
                            }
                        }
                        break;
                    }
                }
            }
            
            maxFlow += pathFlow;
        }
    }
    
    void startAnimation() {
        if (animationPaths.empty()) {
            statusMessage = "No augmenting paths found. Max flow: 0";
            return;
        }
        
        // Get the first path
        currentAnimationPath = animationPaths.front();
        animationPaths.pop();
        
        // Reset all flows to 0 for visualization
        for (auto& edge : edges) {
            edge.updateFlow(0);
        }
        
        // Reset the animation ball
        flowBall.reset(currentAnimationPath[0]);
        flowBall.setPosition(nodes[currentAnimationPath[0]].position);
        
        isAnimating = true;
        statusMessage = "Animating augmenting paths...";
    }

};


int main() {
    int choice = 0;
    showMenu();
    std::cin >> choice;
    
    if (choice == 1) {
        TSPSolver tspSolver;
        tspSolver.run();
    } else if (choice == 2) {
        NetworkFlowSolver networkSolver;
        networkSolver.run();
    } else {
        std::cout << "Invalid choice. Exiting." << std::endl;
    }
    
    return 0;
}