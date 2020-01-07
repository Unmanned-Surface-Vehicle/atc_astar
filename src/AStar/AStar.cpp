#include "../../include/AStar/Astar.h"

namespace AStar
{

  inline double heuristic(Pos a, Pos b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  }

  void AStar::AStarSearch(GridWithWeights graph, 
    Pos start, 
    Pos goal, 
    std::unordered_map<Pos, Pos> &came_from, 
    std::unordered_map<Pos, double> &cost_so_far)
  {
    
    // std::cout << "Started A* Search" << std::endl;

    PriorityQueue<Pos, double> frontier;                                      // Ordering frontier exploration by lowest cost
    frontier.put(start, 0);                                                   // Frontier exploration starts on start point

    came_from[start]    = start;
    cost_so_far[start]  = 0;
    
    // Exploration by lowest cost
    while (!frontier.empty()) { 

      Pos current = frontier.get();                                           // Gets lowest costy position on frontier

      if (current == goal) {                                                  // Early exit when reached the goal
        break;
      }

      for (Pos next : graph.Neighbors(current)) {                             // Frontier expansion
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

    // std::cout << "Finished A* Search" << std::endl;
    // std::cout << "Start: (" << start.x << ", " << start.y << ")" << std::endl;

  }

  // template<typename Pos>
  std::vector<Pos> AStar::reconstruct_path(
    Pos start, Pos goal,
    std::unordered_map<Pos, Pos> came_from)
  {
      
    std::vector<Pos> path;
    Pos current = goal;

    // std::cout << "Goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

    // while ( (current != start) && ((current.x != 0) && (current.y !=0)) )
    while ( current != start )
    {
      path.push_back(current);

      // std::cout << "Current: (" << current.x << ", "  << current.y  << ")" << std::endl;
      // std::cout << "Start: ("   << start.x << ", "    << start.y    << ")" << std::endl;

      // std::unordered_map<Pos, Pos>::const_iterator got = came_from.find (current);
      // if ( got == came_from.end() ){

      //   std::cout << "not found" << std::endl;

      //   std::cout << "Came from size: " << came_from.size() << std::endl;
      //   std::cout << "Came from:" << std::endl;
      //   int counter = 0;
      //   for (auto i = came_from.begin(); i != came_from.end(); i++)
      //   {
      //     std::cout << counter << ": (" << (*i).first.x << ", " << (*i).first.y << ")" << std::endl;
      //     counter++;
      //   }

      //   if (current.x == 0)
      //   {
      //     current.y++;
      //   } 
      //   if (current.y == 0)
      //   {
      //     current.x++;
      //   }

      // }else{
      //   current = came_from[current];
      // }        

      current = came_from[current];

    }
    // std::cout << "Came from size: " << came_from.size() << std::endl;
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;

  }
}; // namespace AStar