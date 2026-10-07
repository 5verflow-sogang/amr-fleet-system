#include "amr_control/core/cte_log.hpp"

#include <cmath>
#include <cstdio>

namespace amr_control
{

const char * classifySegment(double path_curvature, double curve_threshold)
{
  return std::abs(path_curvature) >= curve_threshold ? kSegmentCurve : kSegmentStraight;
}

void writeCteCsvHeader(std::ostream & out)
{
  out << kCteCsvHeader << '\n';
}

void writeCteCsvRow(
  std::ostream & out, const CteSample & sample, double curve_threshold, const std::string & robot)
{
  // 스트림 서식 상태를 건드리지 않도록 snprintf로 만든다
  char buf[512];
  std::snprintf(
    buf, sizeof(buf), "%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,",
    sample.timestamp, sample.planned_x, sample.planned_y, sample.actual_x, sample.actual_y,
    std::abs(sample.cte), classifySegment(sample.path_curvature, curve_threshold));
  out << buf << robot << '\n';
}

}  // namespace amr_control
