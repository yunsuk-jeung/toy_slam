#include <iostream>
#include <tbb/concurrent_unordered_map.h>
#include <Eigen/Dense>

using Map = tbb::concurrent_unordered_map<int, int, std::hash<int>>;

int main() {

  Map map;

  int i = 0;

  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  map.emplace(i, i);
  map.emplace(i, i);
  map.emplace(i, i);

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
  std::cout << I << std::endl;

  for (auto aa = map.begin(); aa != map.end(); aa++) {
    std::cout << aa->first << " / " << aa->second << std::endl;
  }

  for(auto& [key,val] : map ){
    std::cout << key << " / " << val << std::endl;
  }

  return 0;
}