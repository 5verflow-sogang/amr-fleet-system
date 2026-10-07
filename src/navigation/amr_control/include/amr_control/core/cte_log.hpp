#ifndef AMR_CONTROL__CORE__CTE_LOG_HPP_
#define AMR_CONTROL__CORE__CTE_LOG_HPP_

// 하네스 `cte.csv` 기록 (tools/harness/README.md 로그 포맷, #5).
//   명세서 컬럼: timestamp, planned_x, planned_y, actual_x, actual_y, cte
//   부가 컬럼:   segment (straight·curve), robot
// cte는 명세서 정의(계획 경로와 실제 궤적 간 수직 거리)대로 부호 없는 거리 [m]를 쓴다.
// 부호 있는 값은 PathProjection::cte에 있다.
// ROS 비의존 코어 (CONTRIBUTING 4-4).

#include <ostream>
#include <string>

namespace amr_control
{

constexpr const char * kCteCsvHeader =
  "timestamp,planned_x,planned_y,actual_x,actual_y,cte,segment,robot";
constexpr const char * kSegmentStraight = "straight";
constexpr const char * kSegmentCurve = "curve";

struct CteSample
{
  double timestamp = 0.0;
  double planned_x = 0.0;
  double planned_y = 0.0;
  double actual_x = 0.0;
  double actual_y = 0.0;
  // 부호 있는 값을 넣어도 된다. 쓸 때 절댓값으로 바꾼다
  double cte = 0.0;
  // 경로 곡률 [1/m]. segment 분류에 쓴다
  double path_curvature = 0.0;
};

// 곡률이 curve_threshold [1/m] 이상이면 curve, 아니면 straight
const char * classifySegment(double path_curvature, double curve_threshold);

void writeCteCsvHeader(std::ostream & out);

// 한 행. 소수 자릿수는 localization.csv와 같다 (시각 9자리, 거리 6자리)
void writeCteCsvRow(
  std::ostream & out, const CteSample & sample, double curve_threshold, const std::string & robot);

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__CTE_LOG_HPP_
