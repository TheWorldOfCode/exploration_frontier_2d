#ifndef EXPLORATION_FRONTIER_2d_GLOBAL_DEFINITION
#define EXPLORATION_FRONTIER_2d_GLOBAL_DEFINITION

#include <string> 
// Enable debugging 
#define DEBUG 1

namespace exploration 
{ 
  extern bool g_debug;

  extern std::string g_map_frame_id; // The frame id for the map  (used for navigation and gridcells) 

  extern int g_minimum_cluster_size;
}
#endif
