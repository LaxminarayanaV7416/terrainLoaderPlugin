#include <unordered_map>
#include <iostream>
#include <string>
#include <tuple>
#include <cmath>
#include <fstream>
#include <sstream>

struct TripleHash
{
    template <class T1, class T2, class T3>
    std::size_t operator()(const std::tuple<T1, T2, T3>& t) const
    {
        // Hash each element of the tuple
        auto hash1 = std::hash<T1>{}(std::get<0>(t));
        auto hash2 = std::hash<T2>{}(std::get<1>(t));
        auto hash3 = std::hash<T3>{}(std::get<2>(t));
        
        // Combine the three hash values using XOR and bit shifting
        return hash1 ^ (hash2 << 1) ^ (hash3 << 2); // Simple XOR and shifts
    }
};

double roundToSixDecimals(double value) {
    return std::round(value * 1e6) / 1e6;
}


void loadCSVFile(std::string filePath, 
                std::unordered_map<std::tuple<double, double, double>, std::tuple<double, double, double>, TripleHash> &storeDouble)
  {
    std::ifstream file(filePath);

    if (!file.is_open())
    {
      std::cerr << "Could not open the file!" << std::endl;
      return;
    }

    std::string line;
    // Skip the header line if it exists
    std::getline(file, line);

    while (std::getline(file, line))
    {
      std::istringstream ss(line);
      std::string x, y, z, u, v, w;
      std::getline(ss, x, ',');
      std::getline(ss, y, ',');
      std::getline(ss, z, ',');
      std::getline(ss, u, ',');
      std::getline(ss, v, ',');
      std::getline(ss, w, ',');
    //   std::cout << x << ", " << y << ", " << z << "::: "  << u << ", "  << v  << ", " << w << std::endl;
      if (!x.empty() && !y.empty() && !z.empty())
      {
        double double_x = roundToSixDecimals(std::stod(x));
        double double_y = roundToSixDecimals(std::stod(y));
        double double_z = roundToSixDecimals(std::stod(z));
        double double_u, double_v, double_w;
        if(!u.empty()){
            double_u = roundToSixDecimals(std::stod(u));
        } else {
            double_u = 0;
        }
        if(!v.empty()){
            double_v = roundToSixDecimals(std::stod(v));
        } else {
            double_v = 0;
        }
        if(!w.empty()){
            double_w = roundToSixDecimals(std::stod(w));
        } else {
            double_w = 0;
        }
        storeDouble[{double_x, double_y, double_z}] = {double_u, double_v, double_w}; // Add to the map
      }
    }
    file.close();
  }

int main()
{
    // Create an unordered_map with std::tuple<int, int, int> as the key
    std::unordered_map<std::tuple<int, int, int>, std::tuple<double, double, double>, TripleHash> store;
    std::unordered_map<std::tuple<double, double, double>, std::tuple<double, double, double>, TripleHash> storeDouble;
    
    // std::string path = "/home/uav/Documents/terrainLoaderPlugin/trails/wisp_50.csv";
    std::string path = "/home/uav/Documents/terrainLoaderPlugin/trails/windDataPrecisionCoordinates.csv";
    loadCSVFile(path, storeDouble);
    // storeDouble[{0.123456, 0.000345, 1.356565}] = {1.06, 1.06, 1.06};
    int count = 0;
    for (const auto &[key, value] : storeDouble)
    {   
        count++;
        std::cout << std::get<0>(key) << "," 
                  << std::get<1>(key) << "," 
                  << std::get<2>(key)
                  << ": " 
                  << std::get<0>(value) << "," 
                  << std::get<1>(value) << "," 
                  << std::get<2>(value) << std::endl;
    }
    std::cout << "The total records are: " << count << std::endl;
    return 0;
}
