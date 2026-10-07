#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

#include "amr_control/core/cte_log.hpp"

namespace amr_control
{
namespace
{

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

// 하네스 로그 포맷(tools/harness/README.md): 명세서 컬럼 6개 + segment, robot
TEST(CteLog, HeaderMatchesHarnessFormat)
{
  std::ostringstream out;
  writeCteCsvHeader(out);
  const auto fields = splitCsv(out.str().substr(0, out.str().size() - 1));
  const std::vector<std::string> expected{
    "timestamp", "planned_x", "planned_y", "actual_x", "actual_y", "cte", "segment", "robot"};
  EXPECT_EQ(fields, expected);
}

TEST(CteLog, RowFormat)
{
  CteSample sample;
  sample.timestamp = 12.5;
  sample.planned_x = 1.0;
  sample.planned_y = 2.0;
  sample.actual_x = 1.0;
  sample.actual_y = 1.97;
  sample.cte = -0.03;
  sample.path_curvature = 0.0;

  std::ostringstream out;
  writeCteCsvRow(out, sample, 0.1, "amr_01");
  EXPECT_EQ(
    out.str(),
    "12.500000000,1.000000,2.000000,1.000000,1.970000,0.030000,straight,amr_01\n");
}

TEST(CteLog, EmptyRobotKeepsColumnCount)
{
  std::ostringstream out;
  writeCteCsvRow(out, CteSample{}, 0.1, "");
  const std::string line = out.str().substr(0, out.str().size() - 1);
  EXPECT_EQ(splitCsv(line).size(), 8u);
}

TEST(CteLog, SegmentClassification)
{
  EXPECT_STREQ(classifySegment(0.0, 0.1), kSegmentStraight);
  EXPECT_STREQ(classifySegment(0.05, 0.1), kSegmentStraight);
  EXPECT_STREQ(classifySegment(0.1, 0.1), kSegmentCurve);
  EXPECT_STREQ(classifySegment(-0.5, 0.1), kSegmentCurve);
}

}  // namespace
}  // namespace amr_control
