#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <map>
#include <unordered_map>
#include <iostream>
#include <time.h>

struct PairHash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1); // Combine the two hash values
    }
};

int main()
{
    time_t start = time(NULL);

    // defining the unordered_map for the reference to keep already spawned blocks
    std::unordered_map<std::pair<int, int>, double, PairHash> mapping;
    std::ifstream file("/home/uav/Documents/terrainLoaderPlugin/trails/heightMask.csv");

    if (!file.is_open())
    {
        std::cerr << "Could not open the file!" << std::endl;
        return 1;
    }

    std::string line;
    // Skip the header line if it exists
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string x, y, z;

        if (std::getline(ss, x, ',') && std::getline(ss, y, ',') && std::getline(ss, z, ','))
        {
            int int_x = std::stoi(x);
            int int_y = std::stoi(y);
            double double_z = std::stod(z);
            mapping[{int_x, int_y}] =  double_z;// Add to the map
        }
    }

    file.close();
    time_t end = time(NULL);
    std::cout << "total time to load file is " << start - end << std::endl;

    start = time(NULL);
    auto it = mapping.find({8, -219});
    if (it != mapping.end()) {
        // Dereference the iterator to access the value
        std::cout << "Value: " << it->second << std::endl;
    } else {
        std::cout << "Key not found." << std::endl;
    }
    end = time(NULL);
    std::cout << "total time to find key_value is " << start - end << std::endl;

    return 0;
}
