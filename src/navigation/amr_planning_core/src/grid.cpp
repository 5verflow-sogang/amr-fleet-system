#include "amr_planning_core/grid.hpp"

#include <stdexcept>

namespace amr_planning_core
{

Grid::Grid(std::size_t width, std::size_t height, double resolution, const uint8_t * costs)
: width_(width), height_(height), resolution_(resolution), costs_(costs)
{
}

bool Grid::inBounds(const Cell & cell) const noexcept
{
  return cell.x >= 0 && cell.y >= 0 &&
         static_cast<std::size_t>(cell.x) < width_ &&
         static_cast<std::size_t>(cell.y) < height_;
}

std::size_t Grid::index(const Cell & cell) const noexcept
{
  return static_cast<std::size_t>(cell.y) * width_ + static_cast<std::size_t>(cell.x);
}

uint8_t Grid::cost(const Cell & cell) const
{
  if (!inBounds(cell)) {
    throw std::out_of_range("Grid::cost: cell out of bounds");
  }
  return costs_[index(cell)];
}

}  // namespace amr_planning_core
