#ifndef MY_EXCEPTION
#define MY_EXCEPTION

#include <exception> 
#include <string> 

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

struct ParameterNotInServer 
{
  ParameterNotInServer(const std::string m) : _what(m) {}   
  std::string what() const throw()
  {
    return "Can not find parameter " +  _what;
  }   
  private:
  std::string _what;

};
#endif
