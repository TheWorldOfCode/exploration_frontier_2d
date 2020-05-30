
#include "../includes/frontier.h"
#include <exception> 
#include <algorithm>

#include "geometry_msgs/Pose.h" 
#include "ros/ros.h" 

void MergeGroup::insert(int x) 
{
  bool flag = true;
  for(size_t i = 0; i < list.size(); i++ ) {

    if(list[i] == x)
    {
      flag = false;
      break;
    } 

  } 

  if(flag) { 
    list.push_back(x); 
  }
}  

int MergeGroup::at(size_t index) const
{
  return list[index];
}  

size_t MergeGroup::size() const
{
  return list.size(); 
}  

Frontier::Frontier(std::shared_ptr<ros::NodeHandle> h)   
/*************************************************
* See speficiation in the header 
*************************************************/
{
  int tmp;
  if(h->getParam("unknown_marker", tmp))
    unknown_marker = tmp;
  else
    unknown_marker = -1;

  ROS_INFO("Param unknown_marker %i", unknown_marker);

  if(h->getParam("occupied_above", tmp))
     occupied_above = tmp; 
  else
    occupied_above = 25;

  ROS_INFO("Param occipied_above %i", occupied_above);


  explored_area.header.frame_id            = exploration::g_map_frame_id;
  explored_area.header.seq                 = -1;

  frontier_cells.header.frame_id           = exploration::g_map_frame_id;
  frontier_cells.header.seq                = -1;

  frontier_cluster.header.frame_id         = exploration::g_map_frame_id;
  frontier_cluster.header.seq              = -1;

  frontier_cluster_centers.header.frame_id = exploration::g_map_frame_id;
  frontier_cluster_centers.header.seq      = -1;
}


size_t Frontier::search(const Map & map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  vec_frontier_cells.clear(); 

  const uint32_t width    = map.getWidth() ;
  const uint32_t height   = map.getHeight() ;

  std::vector<geometry_msgs::Point> l_frontier_cells;

  frontier_cells.cell_width            = map.getResolution();
  frontier_cells.cell_height           = map.getResolution();
  frontier_cluster_centers.cell_height = map.getResolution();
  frontier_cluster_centers.cell_width  = map.getResolution();


  int x = 0, x_offset = 0,y_offset = 0, z = 0;
  geometry_msgs::Point cell_pos;

  while(y_offset < height) 
  { 

    int8_t current_cell = map.getCellData(x); 



    /***********************************************
     * Detect if cell is a frontier cell           *
     ***********************************************/
    if(isFree(current_cell))
    {
      geometry_msgs::Point cell_pos = map.translateCellInToPosition((x - (width * x_offset)), y_offset );
      int8_t left, right, up, down; 


      if(isUnknown(map.getCellData(x - 1)) ) 
      {
        vec_frontier_cells.push_back(vec2(x - (width * x_offset), y_offset ));

        l_frontier_cells.push_back(cell_pos);
      } 
      else if (isUnknown(map.getCellData(x + 1)))
      {
        vec_frontier_cells.push_back(vec2(x - (width * x_offset), y_offset ));
        l_frontier_cells.push_back(cell_pos);
      } 
      else if (int(x - width) > 0 && isUnknown(map.getCellData(x - width))) 
      {
        vec_frontier_cells.push_back(vec2(x - (width * x_offset), y_offset ));
        l_frontier_cells.push_back(cell_pos);
      } 
      else if ( x + width < width*height && isUnknown(map.getCellData(x + width))) 
      {
        vec_frontier_cells.push_back(vec2(x - (width * x_offset), y_offset ));
        l_frontier_cells.push_back(cell_pos);
      } 
    }  

    /***********************************************
     * Current to next cell                        *
     ***********************************************/
    x++;
    if(x == width * (x_offset + 1)) 
    {
      x_offset++;
      y_offset++; 
    } 
  }

  frontier_cells.cells = l_frontier_cells;
  frontier_cells.header.seq++;

  return vec_frontier_cells.size();

}

size_t Frontier::clustering(size_t minimum_size)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  if(vec_frontier_cells.size() == 0)
    throw std::exception(); 

  clustered_frontier_cells.clear(); 

  std::vector<MergeGroup> merge; 
  for(size_t j = 0; j < vec_frontier_cells.size(); j++ )
  {
    const vec2 p = vec_frontier_cells[j];

    if(clustered_frontier_cells.size() == 0)
    { 
      merge.resize(merge.size() + 1); 
      createCluster(p);  
    }
    else
    {
      std::vector<int> fit;

      for(size_t i = 0; i < clustered_frontier_cells.size(); i++  )
      {
        const vec2 start = clustered_frontier_cells[i].front(); 
        const vec2 end = clustered_frontier_cells[i].back(); 

        const vec2 p_start = p - start;
        const vec2 p_end = p - end;
        if(p_start.squaredLength() < 5 && p_start.getX()  < 2 && p_start.getY() <2 ) 
        {
          fit.push_back(i); 
          clustered_frontier_cells[i].push_back(p); 
        } 
        else if(p_end.squaredLength() < 5 && p_end.getX() < 2 && p_end.getY() < 2  )
        {
          fit.push_back(i); 
          clustered_frontier_cells[i].push_back(p); 
        } 
      }

      if(fit.size() == 0 ) 
      { 
        merge.resize(merge.size() + 1); 
        createCluster(p); 
      }
      else if (fit.size() > 1 ) 
      {

        for(size_t k = 1 ; k < fit.size() ; k++  )
        { 
          merge[fit[0]].insert(fit[k]); 
          merge[fit[k]].insert(fit[0]);
        }

      } 
    } 
  }

  // Merging 
  for(size_t i = 0; i < merge.size(); i++ )
  { 
    std::vector<size_t> base;
    mergeCluster(merge,base,i);
  }

  // Cleanup
  for(size_t i = 0; i < clustered_frontier_cells.size(); i++) 
  {
    if(clustered_frontier_cells[i].size() < minimum_size ) 
    {
      clustered_frontier_cells.erase(clustered_frontier_cells.begin() + i); 
      i--;
    } 

  } 

  // Remove duplicates

  for(size_t i = 0; i < clustered_frontier_cells.size(); i++ ) 
    ROS_INFO("Cluster %i Number of cells %i", (int)i, (int) clustered_frontier_cells[i].size() ) ;

  return clustered_frontier_cells.size(); 
}

size_t Frontier::calcCenter(const Map &  map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  vec_center.clear();
  center.clear();

  for( std::vector<vec2> element : clustered_frontier_cells) 
  {
    vec2 average(0,0) ;
    double size = element.size(); 

    for( const vec2 vec : element) 
    {
      average = average + vec;
    }

    vec_center.push_back(vec2((int) (average.getX() / size) , (int) ( average.getY() / size)) );
    center.push_back(map.translateCellInToPosition(average.getX() / size, average.getY() / size)) ;
  }


  return center.size();
}

vec2 Frontier::findClusterFrontier(const size_t cluster_index)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  vec2 closest;
  const vec2 center = vec_center[cluster_index];

  double squared_distance = std::numeric_limits<double>::max();  


  for( vec2 v : clustered_frontier_cells[cluster_index]) 
  {
    double distance = (v - center).squaredLength();
    if( distance < squared_distance ) 
    {
      squared_distance = distance;
      closest = v;
    } 
  }

   return closest;
}

Frontier::~Frontier()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}

size_t Frontier::getNumberOfCluster() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return clustered_frontier_cells.size(); 
}


nav_msgs::GridCells Frontier::getExploredArea( const Map & map ) 
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  std::vector<geometry_msgs::Point> p;

  if(clustered_frontier_cells.size() > 0 ) 
  {
    for(size_t i = 0; i < clustered_frontier_cells[2].size();i++ ) 
      p.push_back(map.translateCellInToPosition(clustered_frontier_cells[2][i].getX(), clustered_frontier_cells[2][i].getY()) );

    explored_area.cell_height = map.map.info.resolution;
    explored_area.cell_width = map.map.info.resolution;
    explored_area.header.seq++;
  } 

  explored_area.cells  = p;
  return explored_area;
}

nav_msgs::GridCells Frontier::getFrontierCells( ) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return frontier_cells;
}


nav_msgs::GridCells Frontier::getCluster(const Map & map, const size_t cluster)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  std::vector<geometry_msgs::Point> p;

  if(clustered_frontier_cells.size() > 0  && cluster < clustered_frontier_cells.size() ) 
  {
    for(size_t i = 0; i < clustered_frontier_cells[cluster].size();i++ ) 
      p.push_back(map.translateCellInToPosition(clustered_frontier_cells[cluster][i].getX(), clustered_frontier_cells[cluster][i].getY()) );

    frontier_cluster.cell_height = map.map.info.resolution;
    frontier_cluster.cell_width = map.map.info.resolution;
    frontier_cluster.header.seq++;
  } 

  frontier_cluster.cells  = p;
  return frontier_cluster;
}


nav_msgs::GridCells Frontier::getClusterCenterGridCells()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  frontier_cluster_centers.cells = center;
  frontier_cluster_centers.header.seq++;

  return frontier_cluster_centers;

}

std::vector<vec2> Frontier::getClusterCenter() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return vec_center;
}

void Frontier::addClusterCenter(const vec2 c, const geometry_msgs::Point point)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  vec_center.push_back(c); 
  center.push_back(point);
}

// Private functions 

bool Frontier::isFree(int8_t cell) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return cell < occupied_above && cell > -1 ? true : false;
}

bool Frontier::isOccupied(int8_t cell) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return cell > occupied_above ? true : false;
}

bool Frontier::isUnknown(int8_t cell) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return cell == -1 ? true : false;
}

void inline Frontier::createCluster(const vec2 p)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  std::vector<vec2> tmp;
  tmp.push_back(p); 
  clustered_frontier_cells.push_back(tmp); 
}  

void Frontier::mergeCluster(std::vector<MergeGroup> & merge_info, std::vector<size_t> & base, size_t current)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  if(clustered_frontier_cells[current].size() != 0 ) 
  {
    if(merge_info[current].size() > 0 ) 
    {
      std::vector<size_t> new_base = base;
      new_base.push_back(current); 
      for(int i = 0; i < merge_info[current].size(); i++) 
      {
        if(std::find(base.begin(), base.end(), merge_info[current].at(i)) == base.end())
          mergeCluster(merge_info, new_base, merge_info[current].at(i)); 
      } 

      if(base.size() != 0 ) 
        mergeCluster(base.back(), current); 
    }
  } 
}


void Frontier::mergeCluster(size_t base, size_t end)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  for(int i = 0; i < clustered_frontier_cells[end].size(); i++ ) 
    clustered_frontier_cells[base].push_back(clustered_frontier_cells[end][i]); 

  clustered_frontier_cells[end].clear(); 
}


/*
   const double distance_1 = (base_front - end_back).squaredLength();
   const double distance_2 = (base_back  - end_back).squaredLength();
   const double distance_3 = (base_front - end_front).squaredLength();
   const double distance_4 = (base_back  - end_front).squaredLength();

   double distance_tmp;

   ROS_INFO("%f , %f", distance_1 + distance_2, distance_3 + distance_4 );
   if((distance_1 + distance_2)  < (distance_3 + distance_4)) {  
   if(distance_1 < distance_2)
   {
   for(size_t i = 1; i < clustered_frontier_cells[base].size()/2 + 1; i++ ) 
   {
   distance_tmp = (end_back - clustered_frontier_cells[base][i]).squaredLength();   

   if(distance_tmp > distance_1)
   break; 

   vec2 tmp = end_back - clustered_frontier_cells[base][i]; 

   if(tmp.squaredLength() < 5  && tmp.getX() < 2 && tmp.getY() < 2) 
   {
   flag = true;
   break;
   } 
   } 
   }  
   else 
   {
   for(size_t i = clustered_frontier_cells[base].size(); i > clustered_frontier_cells[base].size()/2 - 1; i-- ) 
   {
   distance_tmp = (end_back - clustered_frontier_cells[base][i]).squaredLength();   

   if(distance_tmp > distance_2)
   break; 

   vec2 tmp = end_back - clustered_frontier_cells[base][i]; 

   if(tmp.squaredLength() < 5  && tmp.getX() < 2 && tmp.getY() < 2) 
   {
   flag = true;
   break;
   } 

   } 
   } 
   } 
   else 
   {
   if(distance_1 < distance_2)
   {
   for(size_t i = 1; i < clustered_frontier_cells[base].size()/2 + 1; i++ ) 
   {
   distance_tmp = (end_back - clustered_frontier_cells[base][i]).squaredLength();   

   if(distance_tmp > distance_1)
   break; 

   vec2 tmp = end_back - clustered_frontier_cells[base][i]; 

   if(tmp.squaredLength() < 5  && tmp.getX() < 2 && tmp.getY() < 2) 
   {
   flag = true;
   break;
   } 
   } 
   }  
   else 
   {
   for(size_t i = clustered_frontier_cells[base].size(); i > clustered_frontier_cells[base].size()/2 - 1; i-- ) 
{
  distance_tmp = (end_back - clustered_frontier_cells[base][i]).squaredLength();   

  if(distance_tmp > distance_1)
    break; 

  vec2 tmp = end_back - clustered_frontier_cells[base][i]; 

  if(tmp.squaredLength() < 5  && tmp.getX() < 2 && tmp.getY() < 2) 
  {
    flag = true;
    break;
  } 

} 
} 
} 

if(!flag)
  return; 
  else 
{
  for(int i = 0; i < clustered_frontier_cells[end].size(); i++ ) 
    clustered_frontier_cells[base].push_back(clustered_frontier_cells[end][i]); 
} 
*/
