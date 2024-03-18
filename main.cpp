#include <iostream>
#include "AStar.hpp"

int main() {
    const int width = 100;
    const int height = 60;

    AStar pathfinder(width, height);

    // Set some collisions (obstacles)
    pathfinder.setCollision(0, 0);

    // Set a platform
    pathfinder.setPlatform(1, 2);

    // Set a waterfall
    pathfinder.setWaterfall(1, 2);

    // Set a right_block
    pathfinder.setRightBlock(2, 1);

    // Set a left_block
    pathfinder.setLeftBlock(2, 1);

    // Find a path
    std::vector<std::pair<int, int>> path = pathfinder.findPath(2, 1, 3, 1);

    // Print the path
    std::cout << "Path found: ";
    for (const auto& node : path) {
        std::cout << "(" << node.first << "," << node.second << ") ";
    }
    std::cout << std::endl;

    return 0;
}
