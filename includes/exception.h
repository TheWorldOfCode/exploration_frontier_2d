#ifndef MY_EXCEPTION
#define MY_EXCEPTION

#include <exception> 

struct OutABound : public std::exception 
{
  virtual const char * what() const throw() 
  {
    return "Index not within map";
  }  
};

struct NoFrontier : public std::exception 
{
  virtual const char * what() const throw() 
  {
    return "No frontier cells in map";
  }  
};

struct NoFrontierCluster : public std::exception 
{
  virtual const char * what() const throw() 
  {
    return "No frontier cluster";
  }  
};
#endif
