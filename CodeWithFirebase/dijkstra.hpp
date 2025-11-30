#ifndef DIJ_HPP
#define DIJ_HPP

#include <vector>
#include <utility>

// 高层接口：计算 from → to 的最短路径（节点编号 1..14）
// 返回：
//   result.first  = 节点路径，例如 {1, 4, 10, 11, 14}
//   result.second = 总 cost；如果不可达：路径为空，cost = -1
std::pair<std::vector<int>, int> dijkstra(int from, int to);

#endif