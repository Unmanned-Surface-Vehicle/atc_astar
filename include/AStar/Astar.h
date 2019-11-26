#ifndef ASTAR_H
#define ASTAR_H

// #include "../Pos/Pos.hpp"
#include "../GridWithWeights/GridWithWeights.hpp"
#include "../PriorityQueue/PriorityQueue.hpp"
namespace AStar
{
  class AStar{
    
    public:
      AStar(){};
      ~AStar(){};

      void AStarSearch( GridWithWeights, 
                        Pos, 
                        Pos, 
                        std::unordered_map<Pos, Pos>&, 
                        std::unordered_map<Pos, double>&);

      // template <typename Pos>
      std::vector<Pos> reconstruct_path(Pos, Pos, std::unordered_map<Pos, Pos>);

    private:
  };
}; // namespace AStar

#endif