#ifndef EXPLORATION_HISTOGRAM
#define EXPLORATION_HISTOGRAM

#include <vector>
#include <cmath>
#include <fstream> 
#include <limits>
#include <exception>

// Currently only working for doubles 
template <class T>
class Histogram
{
  public: 
    Histogram() 
    {} 
    Histogram(const std::vector<T> data, const T total = -1, const bool normalized = false) : data(data) , total(total) , normalized(normalized), bins(data.size()) 
    { } 

    /****************************************************
     * Name: normalize                                                  
     * Description: Normalize the histogram
     * Parameters: 
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void normalize( )
    {
      if(total == -1) 
        count(); 


      for(int i=0; i < data.size(); i++)
      {
        data[i] /= total;
      }

      normalized = true;
    }

    /****************************************************
     * Name: count                                                   
     * Description: Count the total number of elements
     * Parameters: 
     * Return:  The count  
     * Throws:  
     * Errors:  
     ****************************************************/
    T count( )
    {
      total = 0;
      for(int i=0; i < data.size() ; i++)
      {
        total += data[i]; 
      }
      return total;
    }

    /****************************************************
     * Name: centerOfMass                                                  
     * Description: calculate the center of mass of the histogram
     * Parameters: 
     * Return:  The index of the center of mass
     * Throws:  
     * Errors:  
     ****************************************************/
    double calcMean() const
    {
      double mean = 0;
      for(int i=0; i < data.size() ; i++)
        mean += data[i]; 

      mean /= data.size();

      return mean;
    }

    /****************************************************
     * Name: max                                                   
     * Description: Return the index of the bin with the max elements
     * Parameters: 
     * Return:  Index
     * Throws:  
     * Errors: 
     ****************************************************/
    int max( ) const
    {

      T max = std::numeric_limits<T>::min(); 
      int index = 0;

      for(int i=0; i < data.size() ; i++)
      {
        if(max < data[i]) 
        {
          max = data[i];
          i = index; 
        } 
      }
    }
    /****************************************************
     * Name: save                                                  
     * Description: save the histogram data
     * Parameters: 
     *            std::string filename - TODO
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void save( std::string filename ) const
    {
      std::ofstream f;
      f.open(filename); 


      if(f.is_open() ) 
      {
        for(int i=0; i < data.size() ; i++)
        {
           f << data[i] << "\n" ;
        }

        f.close(); 
      } 
    }


    /****************************************************
     * Name: getIndexAbove                                                  
     * Description: get the index that contains a values above a given threshold
     * Parameters: 
*                   const double threshold - TODO
     * Return: a list of indexs
     * Throws: 
     * Errors: 
     ****************************************************/
    std::vector<size_t> getIndexAbove(const double threshold) const
    {
      std::vector<size_t> index;

      for(size_t i=0; i < data.size() ; i++)
      {
        if(data[i] > threshold) 
          index.push_back(i); 
      }

      return index;
    }

    /****************************************************
     * Name: getData                                                  
     * Description: Returnes the data at a give index
     * Parameters: 
*                   const int index - TODO
     * Return: Data at index
     * Throws: 
     * Errors: 
     ****************************************************/
    T getData(const int index) const
    {
        if(index < 0 || index >= data.size() ) 
          throw std::out_of_range("Index out of range"); 

        return data[index];
    }
    /****************************************************
     * Name: size                                                   
     * Description:  What does the function do ?
     * Parameters: 
     * Return:  What does the function return ?
     * Throws:  Which exception does it throw ?
     * Errors:  Current errors
     ****************************************************/
    int size( ) const
    {
      return data.size(); 
    }
    ~Histogram()
    { }  

    size_t bins;
  private: 
    std::vector<T> data;
    T total;
    bool normalized;
//    double mean;


};

#endif
