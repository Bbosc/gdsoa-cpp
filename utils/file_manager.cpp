#include "file_manager.hpp"


template <typename T>
void writeVectorsToFile(const std::string filename, const std::vector<T> data)
{
  std::ofstream file(filename);
  if (file.is_open())
  {
    for (T vector: data)
    {
      for (auto value: vector)
      {
        file << value << ',';
      }
      file << '\n';
    }
  }
}

template void writeVectorsToFile<Eigen::Vector3d>(const std::string filename, const std::vector<Eigen::Vector3d> data);
