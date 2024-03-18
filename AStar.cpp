#include "AStar.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

AStar::AStar(int width, int height) : width(width), height(height) {
    grid.resize(width, std::vector<bool>(height, true)); // Initialize grid with all cells walkable
}

void AStar::setCollision(int x, int y) {
    if (isInsideGrid(x, y))
        grid[x][y] = false; // Mark cell as not walkable if inside grid
}

void AStar::setPlatform(int x, int y) {
    if (isInsideGrid(x, y))
        platforms.push_back({ x, y }); // Add platform position to the list
}

void AStar::setWaterfall(int x, int y) {
    if (isInsideGrid(x, y))
        waterfalls.push_back({ x, y }); // Add waterfall position to the list
}

void AStar::setRightBlock(int x, int y) {
    if (isInsideGrid(x, y))
        rightBlocks.push_back({ x, y }); // Add right block position to the list
}

void AStar::setLeftBlock(int x, int y) {
    if (isInsideGrid(x, y))
        leftBlocks.push_back({ x, y }); // Add left block position to the list
}

std::vector<std::pair<int, int>> AStar::findPath(int startX, int startY, int goalX, int goalY) {
    std::vector<std::pair<int, int>> path;

    if (!isInsideGrid(startX, startY) || !isInsideGrid(goalX, goalY))
        return path; // If start or goal is outside grid, return empty path

    if (!isWalkable(startX, startY) || !isWalkable(goalX, goalY))
        return path; // If start or goal is not walkable, return empty path

    std::vector<Node*> openList;
    std::vector<Node*> closedList;

    Node* startNode = new Node(startX, startY);
    Node* goalNode = new Node(goalX, goalY);

    openList.push_back(startNode);

    while (!openList.empty()) {
        Node* currentNode = getNodeWithLowestFCost(openList);

        if (currentNode->x == goalNode->x && currentNode->y == goalNode->y) {
            // Goal reached, construct path
            while (currentNode != nullptr) {
                path.push_back({ currentNode->x, currentNode->y });
                currentNode = currentNode->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        openList.erase(std::remove(openList.begin(), openList.end(), currentNode), openList.end());
        closedList.push_back(currentNode);

        // Generate neighbor nodes
        for (const auto& dir : directions) {
            int neighborX = currentNode->x + dir.first;
            int neighborY = currentNode->y + dir.second;

            if (!isInsideGrid(neighborX, neighborY) || !isWalkable(neighborX, neighborY))
                continue;

            // Check if the neighbor is on a platform
            if (isOnPlatform(neighborX, neighborY)) {
                // If the current node is below the platform, allow movement in all directions
                if (currentNode->y > neighborY) {
                    Node* neighborNode = new Node(neighborX, neighborY);
                    neighborNode->gCost = currentNode->gCost + 1;
                    neighborNode->hCost = calculateHCost(neighborX, neighborY);
                    neighborNode->parent = currentNode;

                    if (std::find_if(closedList.begin(), closedList.end(), [&](Node* node) {
                        return node->x == neighborNode->x && node->y == neighborNode->y;
                        }) != closedList.end())
                        continue;

                        auto openNode = std::find_if(openList.begin(), openList.end(), [&](Node* node) {
                            return node->x == neighborNode->x && node->y == neighborNode->y;
                            });

                        if (openNode == openList.end()) {
                            openList.push_back(neighborNode);
                        }
                        else {
                            if ((*openNode)->gCost > neighborNode->gCost) {
                                (*openNode)->gCost = neighborNode->gCost;
                                (*openNode)->parent = currentNode;
                            }
                        }
                }
            }
            else if (isOnWaterfall(neighborX, neighborY)) {
                // If the current node is below the waterfall, disallow upward movement
                if (currentNode->y > neighborY)
                    continue;

                Node* neighborNode = new Node(neighborX, neighborY);
                neighborNode->gCost = currentNode->gCost + 1;
                neighborNode->hCost = calculateHCost(neighborX, neighborY);
                neighborNode->parent = currentNode;

                if (std::find_if(closedList.begin(), closedList.end(), [&](Node* node) {
                    return node->x == neighborNode->x && node->y == neighborNode->y;
                    }) != closedList.end())
                    continue;

                    auto openNode = std::find_if(openList.begin(), openList.end(), [&](Node* node) {
                        return node->x == neighborNode->x && node->y == neighborNode->y;
                        });

                    if (openNode == openList.end()) {
                        openList.push_back(neighborNode);
                    }
                    else {
                        if ((*openNode)->gCost > neighborNode->gCost) {
                            (*openNode)->gCost = neighborNode->gCost;
                            (*openNode)->parent = currentNode;
                        }
                    }
            }
            else if (isOnRightBlock(neighborX, neighborY)) {
                // If the current node is to the left of the right block, allow movement only to the right
                if (currentNode->x < neighborX) {
                    Node* neighborNode = new Node(neighborX, neighborY);
                    neighborNode->gCost = currentNode->gCost + 1;
                    neighborNode->hCost = calculateHCost(neighborX, neighborY);
                    neighborNode->parent = currentNode;

                    if (std::find_if(closedList.begin(), closedList.end(), [&](Node* node) {
                        return node->x == neighborNode->x && node->y == neighborNode->y;
                        }) != closedList.end())
                        continue;

                        auto openNode = std::find_if(openList.begin(), openList.end(), [&](Node* node) {
                            return node->x == neighborNode->x && node->y == neighborNode->y;
                            });

                        if (openNode == openList.end()) {
                            openList.push_back(neighborNode);
                        }
                        else {
                            if ((*openNode)->gCost > neighborNode->gCost) {
                                (*openNode)->gCost = neighborNode->gCost;
                                (*openNode)->parent = currentNode;
                            }
                        }
                }
            }
            else if (isOnLeftBlock(neighborX, neighborY)) {
                // If the current node is to the right of the left block, allow movement only to the left
                if ((currentNode->x > neighborX)) {
                    Node* neighborNode = new Node(neighborX, neighborY);
                    neighborNode->gCost = currentNode->gCost + 1;
                    neighborNode->hCost = calculateHCost(neighborX, neighborY);
                    neighborNode->parent = currentNode;

                    if (std::find_if(closedList.begin(), closedList.end(), [&](Node* node) {
                        return node->x == neighborNode->x && node->y == neighborNode->y;
                        }) != closedList.end())
                        continue;

                        auto openNode = std::find_if(openList.begin(), openList.end(), [&](Node* node) {
                            return node->x == neighborNode->x && node->y == neighborNode->y;
                            });

                        if (openNode == openList.end()) {
                            openList.push_back(neighborNode);
                        }
                        else {
                            if ((*openNode)->gCost > neighborNode->gCost) {
                                (*openNode)->gCost = neighborNode->gCost;
                                (*openNode)->parent = currentNode;
                            }
                        }
                }
            }
            else {
                Node* neighborNode = new Node(neighborX, neighborY);
                neighborNode->gCost = currentNode->gCost + 1;
                neighborNode->hCost = calculateHCost(neighborX, neighborY);
                neighborNode->parent = currentNode;

                if (std::find_if(closedList.begin(), closedList.end(), [&](Node* node) {
                    return node->x == neighborNode->x && node->y == neighborNode->y;
                    }) != closedList.end())
                    continue;

                    auto openNode = std::find_if(openList.begin(), openList.end(), [&](Node* node) {
                        return node->x == neighborNode->x && node->y == neighborNode->y;
                        });

                    if (openNode == openList.end()) {
                        openList.push_back(neighborNode);
                    }
                    else {
                        if ((*openNode)->gCost > neighborNode->gCost) {
                            (*openNode)->gCost = neighborNode->gCost;
                            (*openNode)->parent = currentNode;
                        }
                    }
            }
        }
    }

    // Clean up memory
    for (Node* node : openList)
        delete node;
    for (Node* node : closedList)
        delete node;

    return path;
}

int AStar::calculateHCost(int x, int y) const {
    // Manhattan distance heuristic
    return std::abs(x - width) + std::abs(y - height);
}

AStar::Node* AStar::getNodeWithLowestFCost(const std::vector<Node*>& nodes) const {
    Node* lowestFCostNode = nodes[0];
    for (Node* node : nodes) {
        if (node->fCost() < lowestFCostNode->fCost())
            lowestFCostNode = node;
    }
    return lowestFCostNode;
}

bool AStar::isInsideGrid(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}

bool AStar::isWalkable(int x, int y) const {
    return grid[x][y];
}

bool AStar::isOnPlatform(int x, int y) const {
    for (const auto& platform : platforms) {
        if (platform.first == x && platform.second == y)
            return true;
    }
    return false;
}

bool AStar::isOnWaterfall(int x, int y) const {
    for (const auto& waterfall : waterfalls) {
        if (waterfall.first == x && waterfall.second == y)
            return true;
    }
    return false;
}

bool AStar::isOnRightBlock(int x, int y) const {
    for (const auto& rightBlock : rightBlocks) {
        if ((rightBlock.first == x && rightBlock.second == y) || ((rightBlock.first + 1) == x && rightBlock.second == y))
            return true;
    }
    return false;
}

bool AStar::isOnLeftBlock(int x, int y) const {
    for (const auto& leftBlock : leftBlocks) {
        if ((leftBlock.first == x && leftBlock.second == y) || ((leftBlock.first - 1) == x && leftBlock.second == y)) {
            return true;
        }
    }
    return false;
}