#include "RRTStar.hpp"
#ifndef INCLUDE_INFORMEDRRTSTAR_HPP_
#define INCLUDE_INFORMEDRRTSTAR_HPP_
class InformedRRTStar : public RRTStar {
 private:
  double cmin_ = 0.0;
  double cmax_ = 0.0;

 public:
  InformedRRTStar();
  ~InformedRRTStar();
  std::vector<double> generateHeuristicNode();
  void runPlanner();
  void resetPlanner();
};
#endif  //  INCLUDE_INFORMEDRRTSTAR_HPP_