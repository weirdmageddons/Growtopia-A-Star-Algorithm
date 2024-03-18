#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <utility>

class AStar {
public:
    AStar(int width, int height);

    void setCollision(int x, int y);
    void setPlatform(int x, int y);
    void setWaterfall(int x, int y);
    void setRightBlock(int x, int y);
    void setLeftBlock(int x, int y);

    std::vector<std::pair<int, int>> findPath(int startX, int startY, int goalX, int goalY);

private:
    struct Node {
        int x, y;
        int gCost, hCost;
        Node* parent;

        Node(int _x, int _y) : x(_x), y(_y), gCost(0), hCost(0), parent(nullptr) {}

        int fCost() const { return gCost + hCost; }
    };

    int calculateHCost(int x, int y) const;
    Node* getNodeWithLowestFCost(const std::vector<Node*>& nodes) const;
    bool isInsideGrid(int x, int y) const;
    bool isWalkable(int x, int y) const;
    bool isOnPlatform(int x, int y) const;
    bool isOnWaterfall(int x, int y) const;
    bool isOnRightBlock(int x, int y) const;
    bool isOnLeftBlock(int x, int y) const;

    int width, height;
    std::vector<std::vector<bool>> grid;
    std::vector<std::pair<int, int>> platforms;
    std::vector<std::pair<int, int>> waterfalls;
    std::vector<std::pair<int, int>> rightBlocks;
    std::vector<std::pair<int, int>> leftBlocks;

    // Define movement directions
    const std::vector<std::pair<int, int>> directions = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };
};

#endif // ASTAR_HPP
