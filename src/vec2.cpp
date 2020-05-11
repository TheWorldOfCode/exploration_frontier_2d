#include "../includes/vec2.h"

using namespace std;

vec2::vec2()
{}  
vec2::vec2(const double xx, const double yy) : x(xx), y(yy)  
{}  

double vec2::length() const
{
  return sqrt( x * x + y * y);
}  

double vec2::squaredLength( ) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   return x*x + y*y;
}

double vec2::manhattanDistance() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return std::abs(x) + std::abs(y);
}

double vec2::getX() const
{
  return x;
}
double vec2::getY() const
{
  return y;
}  

void vec2::normalize()
{

  double l = length(); 

  x /= l;
  y /= l;

}  

void vec2::normalizeX()
{


  x /= x;
  y /= x;

}  

vec2 vec2::round()
{
  return vec2((int)x, (int)y  ); 
}  

vec2 vec2::add(const vec2 b) const
{
  return vec2(x + b.getX() , y + b.getY() );
}  
vec2 vec2::sub(const vec2 b) const
{
  return vec2(x-b.getX(), y-b.getY());
}  
double vec2::cross(const vec2 b) const
{
  return ( x * b.getY() - y * b.getX() );
}  
double vec2::dot(const vec2 b) const
{return x*b.getX() + y * b.getY();
}  
vec2 vec2::scalar(const double b) const
{
  return vec2( x * b , y * b );
}  

bool vec2::compare(const vec2 b) const
{
     return x == b.getX() && y == b.getY();  
} 

vec2::~vec2()
{} 


vec2 vec2::operator+(const vec2 b) const
{
  return add(b);
}  
vec2 vec2::operator-(const vec2 b) const
{
  return sub(b);
}
double vec2::operator*(const vec2 b) const
{
  return dot(b);
}
vec2 vec2::operator*(const double b) const
{ 
  return scalar(b);
}  

bool vec2::operator==(const vec2 b) const
{
  return compare(b); 
} 

bool vec2::operator!=(const vec2 b) const
{
  return !compare(b); 
} 

std::string vec2::print() const
{
  return  "( " + std::to_string(x) +  " , " + std::to_string(y) + " )";   

} 

ostream & operator<<(ostream & os, vec2 & v)
{

  return os << "( " << v.x << " , " << v.y << " )";   

}
