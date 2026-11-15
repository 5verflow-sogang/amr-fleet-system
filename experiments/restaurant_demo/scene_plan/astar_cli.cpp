// amr_planning_core A*를 비용 격자 파일에 돌린다 (1-2 우회로 확인용).
//   astar_cli <grid.bin> <sx> <sy> <gx> <gy> > path.txt
// grid.bin: uint32 width, uint32 height, float64 resolution, width*height uint8 (행 y=0이 아래쪽)
// 설정은 nav2_params.yaml AStar 블록과 같다: octile, lethal 253, cost_weight 4.0, prevent_corner_cutting
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include "amr_planning_core/astar.hpp"
#include "amr_planning_core/grid.hpp"

using amr_planning_core::AStarConfig;
using amr_planning_core::AStarPlanner;
using amr_planning_core::Cell;
using amr_planning_core::Grid;
using amr_planning_core::HeuristicType;
using amr_planning_core::PlanStatus;

int main(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr << "usage: astar_cli grid.bin sx sy gx gy\n";
    return 2;
  }
  std::ifstream f(argv[1], std::ios::binary);
  uint32_t w = 0, h = 0;
  double res = 0.0;
  f.read(reinterpret_cast<char *>(&w), 4);
  f.read(reinterpret_cast<char *>(&h), 4);
  f.read(reinterpret_cast<char *>(&res), 8);
  std::vector<uint8_t> costs(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char *>(costs.data()), costs.size());
  Grid grid(w, h, res, costs.data());
  AStarConfig cfg;
  cfg.heuristic = HeuristicType::kOctile;
  cfg.lethal_cost_threshold = 253;
  cfg.cost_weight = 4.0;
  cfg.prevent_corner_cutting = true;
  AStarPlanner planner(cfg);
  Cell s{std::atoi(argv[2]), std::atoi(argv[3])};
  Cell g{std::atoi(argv[4]), std::atoi(argv[5])};
  auto r = planner.plan(grid, s, g);
  std::cout << "status " << static_cast<int>(r.status) << "\n";
  for (const auto & c : r.path) {
    std::cout << c.x << " " << c.y << "\n";
  }
  return r.status == PlanStatus::kSuccess ? 0 : 1;
}
