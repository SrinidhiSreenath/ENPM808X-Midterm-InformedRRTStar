#include "include/RRTStar.hpp"

class InformedRRTStar : public RRTStar
{
  private:
    double cmin_;
    double cmax_;

  public:
    bool isHeuristicNode(const std::vector<double> &newNode);
};