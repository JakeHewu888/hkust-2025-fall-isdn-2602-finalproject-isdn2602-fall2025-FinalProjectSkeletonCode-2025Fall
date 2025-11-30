#include "dijkstra.hpp"
#include <vector>
#include <climits>
#include <algorithm>
#include <Arduino.h>

#define NUM_NODES 14
#define INF 10000

const int graph[NUM_NODES][NUM_NODES] = {
    //  1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14
    {0, 9, INF, 4, INF, INF, INF, INF, INF, INF, INF, INF, INF, INF},   // 1
    {9, 0, 9, INF, 4, INF, INF, INF, INF, INF, INF, INF, INF, INF},     // 2
    {INF, 9, 0, INF, INF, INF, INF, INF, INF, INF, 14, INF, INF, INF},  // 3
    {4, INF, INF, 0, 9, INF, INF, INF, INF, 9, INF, INF, INF, INF},     // 4
    {INF, 4, INF, 9, 0, 4, INF, 4, INF, INF, INF, INF, INF, INF},       // 5
    {INF, INF, INF, INF, 4, 0, INF, INF, INF, INF, INF, INF, INF, INF}, // 6
    {INF, INF, INF, INF, INF, INF, 0, 4, INF, INF, INF, INF, INF, INF}, // 7
    {INF, INF, INF, INF, 4, INF, 4, 0, 4, INF, INF, INF, 9, INF},       // 8
    {INF, INF, INF, INF, INF, INF, INF, 4, 0, INF, INF, INF, INF, INF}, // 9
    {INF, INF, INF, 9, INF, INF, INF, INF, INF, 0, 2, 4, INF, INF},     // 10
    {INF, INF, 14, INF, INF, INF, INF, INF, INF, 2, 0, INF, INF, 4},    // 11
    {INF, INF, INF, INF, INF, INF, INF, INF, INF, 4, INF, 0, 9, INF},   // 12
    {INF, INF, INF, INF, INF, INF, INF, 9, INF, INF, INF, 9, 0, 9},     // 13
    {INF, INF, INF, INF, INF, INF, INF, INF, INF, INF, 4, INF, 9, 0}    // 14
};

// Dijkstra
// 计算从 startNode 到 endNode 的最短路径
// startNode、endNode 都是 1~NUM_NODES
// path[] 里存放路径上的节点编号（也是 1~NUM_NODES）
// pathLength 为路径长度（path 里实际用到的元素个数）
// totalCost 是最短路径的总 cost
void dijkstra(int startNode, int endNode, int *path, int &pathLength, int &totalCost)
{
    // 内部用 0-based 索引
    int start = startNode - 1;
    int end = endNode - 1;

    int dist[NUM_NODES];
    bool visited[NUM_NODES];
    int parent[NUM_NODES];

    // 初始化
    for (int i = 0; i < NUM_NODES; i++)
    {
        dist[i] = INF;
        visited[i] = false;
        parent[i] = -1;
    }
    dist[start] = 0;

    // 主循环：O(N^2) 完全没问题，节点才 14 个
    for (int i = 0; i < NUM_NODES - 1; i++)
    {
        // 1. 找到当前未访问节点中 dist 最小的那个
        int u = -1;
        int minDist = INF;
        for (int j = 0; j < NUM_NODES; j++)
        {
            if (!visited[j] && dist[j] < minDist)
            {
                minDist = dist[j];
                u = j;
            }
        }

        // 如果找不到可扩展节点，提前结束
        if (u == -1)
            break;

        visited[u] = true;

        // 2. 用 u 去更新所有邻居 v
        for (int v = 0; v < NUM_NODES; v++)
        {
            // graph[u][v] < INF 表示有边
            if (!visited[v] && graph[u][v] < INF)
            {
                int newDist = dist[u] + graph[u][v];
                if (newDist < dist[v])
                {
                    dist[v] = newDist;
                    parent[v] = u;
                }
            }
        }
    }

    // 没有路径
    if (dist[end] == INF)
    {
        pathLength = 0;
        totalCost = INF;
        return;
    }

    // 3. 回溯 parent[] 重构路径（先倒着存，再反转）
    int tempPath[NUM_NODES];
    int tempLen = 0;

    int current = end;
    while (current != -1)
    {
        tempPath[tempLen++] = current;
        if (current == start)
            break;
        current = parent[current];
    }

    // 反转并转换回 1-based 节点编号
    pathLength = 0;
    for (int i = tempLen - 1; i >= 0; i--)
    {
        path[pathLength++] = tempPath[i] + 1; // +1 变回 1~NUM_NODES
    }

    totalCost = dist[end];
}

// 高层封装：返回 std::vector<int> 路径 + cost
std::pair<std::vector<int>, int> dijkstra(int from, int to)
{
    int rawPath[NUM_NODES];
    int pathLen = 0;
    int totalCost = 0;

    // 调用下层实现
    dijkstra(from, to, rawPath, pathLen, totalCost);

    std::vector<int> path;
    for (int i = 0; i < pathLen; i++)
    {
        path.push_back(rawPath[i]); // rawPath 里是 1-based 的 node id
    }

    if (pathLen == 0)
    {
        // 不可达时，规范 cost = -1，方便上层判断
        totalCost = -1;
    }

    return {path, totalCost};
}