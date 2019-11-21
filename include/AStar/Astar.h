#ifndef ASTAR_H
#define ASTAR_H

// #include "../Posi/Posi.hpp"
#include "../GridWithWeights/GridWithWeights.hpp"
#include "../PriorityQueue/PriorityQueue.hpp"
namespace AStar
{
  class AStar{
    
    public:
      AStar(){};
      ~AStar(){};

      void AStarSearch( GridWithWeights, 
                        Posi, 
                        Posi, 
                        std::unordered_map<Posi, Posi>&, 
                        std::unordered_map<Posi, double>&);

      // template <typename Posi>
      std::vector<Posi> reconstruct_path(Posi, Posi, std::unordered_map<Posi, Posi>);

    private:
  };
}; // namespace AStar

#endif