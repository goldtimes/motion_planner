# 全局路径规划算法笔记

## Dijkstra 算法

```
function Dijkstra(Graph, source):
    dist[source] ← 0
    for each vertex v in Graph:
        if v ≠ source
            dist[v] ← ∞
        add v to Q    // 未处理顶点集合
    
    while Q is not empty:
        u ← vertex in Q with min dist[u]
        remove u from Q
        
        for each neighbor v of u:
            alt ← dist[u] + length(u, v)
            if alt < dist[v]:
                dist[v] ← alt
                prev[v] ← u
    
    return dist[], prev[]
```

**特点**
- 广度优先搜索 + 权值累加
- 保证找到最短路径
- 时间复杂度：$O(V^2)$ 或 $O((V+E)\log V)$（使用优先队列）

---

## A\* 算法

```
function AStar(Graph, start, goal, h):
    openSet ← {start}
    gScore[start] ← 0
    fScore[start] ← h(start)
    
    while openSet is not empty:
        current ← node in openSet with min fScore
        if current == goal:
            return reconstruct_path(cameFrom, current)
        
        openSet.remove(current)
        closedSet.add(current)
        
        for each neighbor of current:
            if neighbor in closedSet:
                continue
            tentative_g ← gScore[current] + d(current, neighbor)
            if neighbor not in openSet:
                openSet.add(neighbor)
            elif tentative_g ≥ gScore[neighbor]:
                continue
            
            cameFrom[neighbor] ← current
            gScore[neighbor] ← tentative_g
            fScore[neighbor] ← gScore[neighbor] + h(neighbor)
    
    return failure
```

**启发函数 $h(n)$ 的选择**
- 欧氏距离：$h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}$
- 曼哈顿距离：$h(n) = |x_n - x_g| + |y_n - y_g|$
- 对角线距离：$h(n) = \max(|x_n - x_g|, |y_n - y_g|)$

**特点**
- 引入启发函数引导搜索方向
- 如果 $h(n) \le$ 实际代价（可采纳），保证最优解
- 效率通常优于 Dijkstra

---

## 对比

| 特性 | Dijkstra | A* | RRT |
|------|----------|-----|-----|
| 最优性 | ✅ 保证 | ✅ 保证（可采纳启发函数）| ❌ 不保证 |
| 完备性 | ✅ | ✅ | ✅（概率完备）|
| 效率 | 低（遍历多）| 中（启发式剪枝）| 高（采样）|
| 适用场景 | 静态图 | 静态图 | 高维/非完整约束 |
| ROS 实现 | `navfn` | `global_planner` | 需自行实现 |
