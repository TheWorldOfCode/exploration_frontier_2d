#ifndef FRONTIER
#define FRONTIER

// ROS
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/GridCells.h" 

// MY
#include "../includes/global_definition.h" 
#include "../includes/map.h" 
#include "../includes/vec2.h" 

// CPP STANDARD
#include <vector>

class MergeGroup 
{
  public:
    std::vector<int> list;
    void insert(int x);
    int at(size_t index) const;
    size_t size() const;
};

class Frontier 
{
  public:
    Frontier(int8_t unknown_marker, int8_t occupied_above);
    /****************************************************
     * Name: search                                                  
     * Description:  Locate frontier cell in the map
     * Parameters: 
     *            map - The Current map
     * Return: Number of Frontier cells
     * Throws:  
     * Errors: NO 
     ****************************************************/
    size_t search(const Map & map);

    /****************************************************
     * Name: clustering                                                  
     * Description: Cluster the frontier cells
     * Parameters: 
     *            minimum_size - minimum threshold of number of cells in a cluster
     * Return: Number of cluster created
     * Throws: 
     * Errors: Not merging all adjacent clusters together
     ****************************************************/
    size_t clustering(size_t minimum_size = 5);

    /****************************************************
     * Name: calcCenter                                                  
     * Description: Calculate the center of the clusters
     * Parameters: 
*                   const Map & map - TODO
     * Return: The number of centers
     * Throws: 
     * Errors: 
     ****************************************************/
    size_t calcCenter(const Map & map);


    ~Frontier(); 

    /****************************************************
     * Name: getNumberOfCluster                                                  
     * Description:  Return the number of cluster detected
     * Parameters: 
     * Return:  The number of cluster
     * Throws:  
     * Errors:  
     ****************************************************/
    size_t getNumberOfCluster() const;

    /****************************************************
     * Name: getExploredArea                                                  
     * Description: Returns the Grid cells that are explored 
     * Parameters: 
     * Return:  Grid cells
     * Throws:  
     * Errors: 
     ****************************************************/
    nav_msgs::GridCells getExploredArea(const Map & map);

    /****************************************************
     * Name: getFrontierCells                                                  
     * Description: Return the frontier cells 
     * Parameters: 
     * Return:  Return the frontier cells
     * Throws:  
     * Errors:  
     ****************************************************/
    nav_msgs::GridCells getFrontierCells() const;


    /****************************************************
     * Name: getCluster                                                  
     * Description: Convert a cluster into a gridcells
     * Parameters: 
*                   map - TODO
*                   cluster - TODO
     * Return: Gridcells in the world coordinates
     * Throws: 
     * Errors: 
     ****************************************************/
    nav_msgs::GridCells getCluster(const Map & map, const size_t cluster);

    /****************************************************
     * Name: getClusterCenterGridCells
     * Description: Get the cluster centers in a gridcell
     * Parameters: 
     * Return:  Returns a gridcell with the cluster centers
     * Throws: 
     * Errors:
     ****************************************************/
    nav_msgs::GridCells getClusterCenterGridCells();

    /****************************************************
     * Name: getClusterCenter(                                                  
     * Description: Return a list of cluster center in map coordinate system
     * Parameters: 
     * Return: Return a list of cluster centers
     * Throws: 
     * Errors:  
     ****************************************************/
    std::vector<vec2> getClusterCenter() const;
    void addClusterCenter(const vec2 center); 

    /****************************************************
     * Name: isFree                                                  
     * Description: Check if a cell is free space
     * Parameters: 
     *                   int8_t cell - Map cell
     * Return:  True if the cell is free space
     * Throws:  
     * Errors: 
     ****************************************************/
    bool isFree(int8_t cell) const;

    /****************************************************
     * Name: isOccupied                                                  
     * Description: Check if a cell is occupied
     * Parameters: 
     *                   int8_t cell - Map cell
     * Return: True if cell is occupied
     * Throws:
     * Errors: 
     ****************************************************/
    bool isOccupied(int8_t cell) const;

    /****************************************************
     * Name: isUnknown                                                  
     * Description: Check if a cell is unknown
     * Parameters: 
     *                   int8_t map - Map cell
     * Return:  True if cell is unknown       
     * Throws: 
     * Errors:
     ****************************************************/
    bool isUnknown(int8_t) const;

  private:

    bool g_debug;
    nav_msgs::GridCells explored_area;
    nav_msgs::GridCells frontier_cells;
    nav_msgs::GridCells frontier_cluster;
    nav_msgs::GridCells frontier_cluster_centers;
    std::vector<geometry_msgs::Point> center; 

    int8_t unknown_marker;
    int8_t occupied_above;

    std::vector<vec2> vec_frontier_cells; // Contains inclusteres frontier cells
    std::vector< std::vector<vec2> > clustered_frontier_cells; // Clustered frontier cells
    std::vector<vec2> vec_center; // Contains the center of the frontier clusteres 


    /****************************************************
     * Name: createCluster                                                  
     * Description: Create a new cluster
     * Parameters: 
     *                   geometry_msgs::Point p - The starting point for the cluster
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void inline createCluster(const vec2 p);


    /****************************************************
     * Name: mergeCluster                                                  
     * Description: Merging cluster together 
     * Parameters: 
     *                   std::vector<MergeGroup> & merge_info - TODO
     *                   std::vector<size_t> & base - TODO
     *                   size_t & current - TODO
     * Return:
     * Throws: 
     * Errors:
     ****************************************************/
    void mergeCluster(std::vector<MergeGroup> & merge_info, std::vector<size_t> & base, size_t current);


    /****************************************************
     * Name: mergeCluster                                                  
     * Description: Merge two cluster together, clears the cluster indexed by end
     * Parameters: 
     *                   size_t base - TODO
     *                   size_t end - TODO
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void mergeCluster(size_t base, size_t end);
};

#endif
