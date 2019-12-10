#ifndef GRIDWITHWEIGHTS_HPP
#define GRIDWITHWEIGHTS_HPP

#include "../SquareGrid/SquareGrid.hpp"

struct GridWithWeights: SquareGrid {
  std::unordered_set<Pos> forests;
  GridWithWeights(int w, int h): SquareGrid(w, h) {}
  double cost(Pos from_node, Pos to_node) const {
    return 1; //forests.find(to_node) != forests.end()? 5 : 1;
  }
};

// inline GridWithWeights make_diagram4() {
//   GridWithWeights grid(20, 20);
//   add_rect(grid, 1, 7, 4, 9);
//   typedef Pos L;
//   grid.forests = std::unordered_set<Pos> {
//     L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
//     L{4, 3}, L{4, 4}, L{4, 5}, L{4, 6},
//     L{4, 7}, L{4, 8}, L{5, 1}, L{5, 2},
//     L{5, 3}, L{5, 4}, L{5, 5}, L{5, 6},
//     L{5, 7}, L{5, 8}, L{6, 2}, L{6, 3},
//     L{6, 4}, L{6, 5}, L{6, 6}, L{6, 7},
//     L{7, 3}, L{7, 4}, L{7, 5}
//   };
//   return grid;
// }

#endif