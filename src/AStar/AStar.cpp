#include "../../include/AStar/Astar.h"

namespace AStar
{

  inline double heuristic(Posi a, Posi b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  }

  void AStar::AStarSearch(GridWithWeights graph, 
    Posi start, 
    Posi goal, 
    std::unordered_map<Posi, Posi> &came_from, 
    std::unordered_map<Posi, double> &cost_so_far)
  {
    PriorityQueue<Posi, double> frontier;                                      // Ordering frontier exploration by lowest cost
    frontier.put(start, 0);                                                   // Frontier exploration starts on start point

    came_from[start]    = start;
    cost_so_far[start]  = 0;
    
    // Exploration by lowest cost
    while (!frontier.empty()) { 

      Posi current = frontier.get();                                           // Gets lowest costy position on frontier

      if (current == goal) {                                                  // Early exit when reached the goal
        break;
      }

      for (Posi next : graph.Neighbors(current)) {                             // Frontier expansion
        double new_cost = cost_so_far[current] + graph.cost(current, next);   // Add information about the cost of new frontier's position cost
        if (cost_so_far.find(next) == cost_so_far.end()                       // Posiition not considered yet
            || new_cost < cost_so_far[next]) {                                // During a expansion comming from another path, the cost can be lower, so we updated our data structures wih this information
          cost_so_far[next] = new_cost;
          double priority   = new_cost + heuristic(next, goal);
          frontier.put(next, priority);
          came_from[next]   = current;        
        }
      }
    }
  }

  // template<typename Posi>
  std::vector<Posi> AStar::reconstruct_path(
    Posi start, Posi goal,
    std::unordered_map<Posi, Posi> came_from)
  {
      
    std::vector<Posi> path;
    Posi current = goal;
    while (current != start) {
      path.push_back(current);
      current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;

  }
}; // namespace AStar