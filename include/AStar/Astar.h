#ifndef ASTAR_H
#define ASTAR_H

// #include "../Posi/Posi.hpp"
#include "../GridWithWeights/GridWithWeights.hpp"
#include "../PriorityQueue/PriorityQueue.hpp"
namespace AStar
{
  class AStar{
    
    public:
      AStar();
      ~AStar();

      void AStarSearch(GridWithWeights graph,
                              Posi start,
                              Posi goal,
                              std::unordered_map<Posi, Posi> &came_from,
                              std::unordered_map<Posi, double> &cost_so_far);
      template <typename Posi>
      std::vector<Posi> reconstruct_path(Posi, Posi, std::unordered_map<Posi, Posi>);

    private:
  };
}; // namespace AStar

#endif