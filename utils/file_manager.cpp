#include "file_manager.hpp"



template <typename T>
bool writeVectorsToFile(const std::string filename, const std::vector<T> data)
{
  std::ofstream file(filename);
  if (file.is_open())
  {
    for (T vector: data)
    {
      file << vector << '\n';
    }
  }
  return 0;
}
