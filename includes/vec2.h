#ifndef VEC2
#define VEC2

#include<cmath>
#include<iostream>
#include<string>


class vec2 {

  public:
    vec2(); 
    vec2(const double xx, const double yy); 
    double length() const; 
    /****************************************************
     * Name: squaredLength
     * Description: Calculate the squared euclidean distance 
     * Parameters: 
     * Return:  The squared euclidean distance 
     * Throws:  
     * Errors: 
     ****************************************************/
    double squaredLength() const;
    double manhattanDistance() const;

    double getX() const;
    double getY() const;  

    void normalize(); 
    void normalizeX(); 
    vec2 round(); 

    vec2 add(const vec2 b) const;
    vec2 sub(const vec2 b) const;
    double cross(const vec2 b) const;
    double dot(const vec2 b) const; 
    vec2 scalar(const double b) const;  

    bool compare(const vec2 b) const; 

    ~vec2(); 

    vec2 operator+(const vec2 b) const;
    vec2 operator-(const vec2 b) const;   
    double operator*(const vec2 b) const;
    vec2 operator*(const double b) const;   

    bool operator==(const vec2 b) const;
    bool operator!=(const vec2 b) const;

    std::string print() const; 
  private: 
    double x;
    double y;

    friend std::ostream & operator<<(std::ostream & os, vec2 & v); 

}; 


std::ostream & operator<<(std::ostream & os, vec2 & v);

#endif
