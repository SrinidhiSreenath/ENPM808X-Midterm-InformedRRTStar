#include <utility>
#include <vector>

class Map {
 public:
  Map::Map() {}
  Map::~Map() {}

  std::vector<std::vector<double>> obstacleList;
  std::vector<std::pair<double, double>> workspaceBoundary;
  std::vector<std::pair<double, double>> halfPlanes;

  void addObstacle(const std::vector<double> &obstacle);
  void calculateHalfPlanes();
  void resetMap();
};