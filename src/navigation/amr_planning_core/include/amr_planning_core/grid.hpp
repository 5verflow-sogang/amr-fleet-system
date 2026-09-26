#ifndef AMR_PLANNING_CORE__GRID_HPP_
#define AMR_PLANNING_CORE__GRID_HPP_

#include <cstddef>
#include <cstdint>

namespace amr_planning_core
{

struct Cell
{
  int x = 0;
  int y = 0;
};

inline bool operator==(const Cell & lhs, const Cell & rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool operator!=(const Cell & lhs, const Cell & rhs)
{
  return !(lhs == rhs);
}

// 2D 점유 격자의 읽기 전용 뷰. 비용 버퍼는 소유하지 않는다.
// 2주차에 nav2_core 플러그인에서 costmap의 getCharMap() 버퍼를 복사 없이
// 그대로 감싸기 위한 설계다. 따라서 plan() 호출이 끝날 때까지 버퍼는
// 호출자(costmap 또는 테스트 코드)가 유지해야 한다.
class Grid
{
public:
  Grid(std::size_t width, std::size_t height, double resolution, const uint8_t * costs);

  std::size_t width() const noexcept {return width_;}
  std::size_t height() const noexcept {return height_;}
  double resolution() const noexcept {return resolution_;}

  bool inBounds(const Cell & cell) const noexcept;

  // cell이 범위 밖이면 예외를 던진다. 호출 전 inBounds()로 먼저 확인한다.
  uint8_t cost(const Cell & cell) const;

private:
  std::size_t index(const Cell & cell) const noexcept;

  std::size_t width_;
  std::size_t height_;
  double resolution_;
  const uint8_t * costs_;
};

}  // namespace amr_planning_core

#endif  // AMR_PLANNING_CORE__GRID_HPP_
