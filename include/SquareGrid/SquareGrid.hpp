#ifndef SQUAREGRID_HPP
#define SQUAREGRID_HPP

#include <unordered_set>    //  walls
#include <vector>           //  Nodes's neighbors
#include <algorithm>        //  std::reverse (used as aesthetic to show the found path)
#include <iostream>         //  std::cout std::left
#include <iomanip>          //  std::setw (draw_grid)

#include "../Posi/Posi.hpp"

// Possible moviment's directions
// static std::array<Pos, 4> DIRS = {Pos{1, 0}, Pos{0, -1}, Pos{-1, 0}, Pos{0, 1}};
static std::array<Posi, 8> DIRS = {Posi{0, 1}, Posi{1, 1}, Posi{1, 0}, Posi{1, -1}, Posi{0, -1}, Posi{-1, -1}, Posi{-1, 0}, Posi{-1, -1}};

/*
*   Squared grid:
*       - Size: width x height
*       - Compounded by walls (borders and obstacles inside the grid)
*/
struct SquareGrid {
    
    int width, height;
    std::unordered_set<Posi> walls;

    SquareGrid(int width_, int height_) : width(width_), height(height_) {}

    bool in_bounds(Posi id) const {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }

    bool passable(Posi id) const {
        return walls.find(id) == walls.end();
    }

    std::vector<Posi> Neighbors(Posi id) const {
        std::vector<Posi> results;

        for (Posi dir : DIRS) {
            Posi next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0) {
            // aesthetic improvement on square grids
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
inline void draw_grid(const SquareGrid& grid, int field_width,
               std::unordered_map<Posi, double>* distances=nullptr,
               std::unordered_map<Posi, Posi>* point_to=nullptr,
               std::vector<Posi>* path=nullptr) {
  for (int y = 0; y != grid.height; ++y) {
    for (int x = 0; x != grid.width; ++x) {
      Posi id {x, y};
      std::cout << std::left << std::setw(field_width);
      if (grid.walls.find(id) != grid.walls.end()) {
        std::cout << std::string(field_width, '#');
      } else if (point_to != nullptr && point_to->count(id)) {
        Posi next = (*point_to)[id];
        if (next.x == x + 1) { std::cout << "> "; }
        else if (next.x == x - 1) { std::cout << "< "; }
        else if (next.y == y + 1) { std::cout << "v "; }
        else if (next.y == y - 1) { std::cout << "^ "; }
        else { std::cout << "* "; }
      } else if (distances != nullptr && distances->count(id)) {
        std::cout << (*distances)[id];
      } else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
        std::cout << '@';
      } else {
        std::cout << '.';
      }
    }
    std::cout << '\n';
  }
}

inline void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
  for (int x = x1; x < x2; ++x) {
    for (int y = y1; y < y2; ++y) {
      grid.walls.insert(Posi{x, y});
    }
  }
}

// SquareGrid make_diagram1() {
//   SquareGrid grid(30, 15);
//   add_rect(grid, 3, 3, 5, 12);
//   add_rect(grid, 13, 4, 15, 15);
//   add_rect(grid, 21, 0, 23, 7);
//   add_rect(grid, 23, 5, 26, 7);
//   return grid;
// }

#endif