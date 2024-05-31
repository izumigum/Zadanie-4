#include <iostream>
#include <queue>
using namespace std;

const int INF = 1e9;

class Graph {
private:
    int V; // Количество вершин
    int** adjMatrix; // Матрица смежности

public:
    Graph(int vertices) : V(vertices) {
        adjMatrix = new int* [V];
        for (int i = 0; i < V; ++i) {
            adjMatrix[i] = new int[V];
            for (int j = 0; j < V; ++j) {
                adjMatrix[i][j] = (i == j) ? 0 : INF;
            }
        }
    }

    ~Graph() {
        for (int i = 0; i < V; ++i) {
            delete[] adjMatrix[i];
        }
        delete[] adjMatrix;
    }

    void addEdge(int u, int v, int weight) {
        adjMatrix[u][v] = weight;
        adjMatrix[v][u] = weight;
    }

    void dijkstra(int src, int* dist) {
        for (int i = 0; i < V; ++i) {
            dist[i] = INF;
        }
        dist[src] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({ 0, src });

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (int v = 0; v < V; ++v) {
                if (adjMatrix[u][v] != INF && dist[u] + adjMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + adjMatrix[u][v];
                    pq.push({ dist[v], v });
                }
            }
        }
    }

    void prim(int** mst) {
        int* key = new int[V];
        int* parent = new int[V];
        bool* inMST = new bool[V];

        for (int i = 0; i < V; ++i) {
            key[i] = INF;
            parent[i] = -1;
            inMST[i] = false;
        }

        key[0] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({ 0, 0 });

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            inMST[u] = true;

            for (int v = 0; v < V; ++v) {
                if (adjMatrix[u][v] != INF && !inMST[v] && adjMatrix[u][v] < key[v]) {
                    key[v] = adjMatrix[u][v];
                    pq.push({ key[v], v });
                    parent[v] = u;
                }
            }
        }

        for (int i = 1; i < V; ++i) {
            mst[i][0] = parent[i];
            mst[i][1] = i;
            mst[i][2] = adjMatrix[i][parent[i]];
        }

        delete[] key;
        delete[] parent;
        delete[] inMST;
    }

    void kruskal(int** mst, int& edgeCount) {
        edgeCount = 0;
        pair<int, pair<int, int>>* edges = new pair<int, pair<int, int>>[V * (V - 1) / 2];
        int edgeIndex = 0;

        for (int u = 0; u < V; ++u) {
            for (int v = u + 1; v < V; ++v) {
                if (adjMatrix[u][v] != INF) {
                    edges[edgeIndex++] = { adjMatrix[u][v], {u, v} };
                }
            }
        }

        // Пузырьковая сортировка
        for (int i = 0; i < edgeIndex; ++i) {
            for (int j = 0; j < edgeIndex - 1; ++j) {
                if (edges[j].first > edges[j + 1].first) {
                    swap(edges[j], edges[j + 1]);
                }
            }
        }

        int* parent = new int[V];
        for (int i = 0; i < V; ++i) {
            parent[i] = i;
        }

        auto find = [&](int u) {
            while (parent[u] != u) u = parent[u];
            return u;
            };

        for (int i = 0; i < edgeIndex; ++i) {
            int u = edges[i].second.first;
            int v = edges[i].second.second;
            int set_u = find(u);
            int set_v = find(v);
            if (set_u != set_v) {
                mst[edgeCount][0] = u;
                mst[edgeCount][1] = v;
                mst[edgeCount][2] = edges[i].first;
                ++edgeCount;
                parent[set_u] = set_v;
            }
        }

        delete[] edges;
        delete[] parent;
    }
};

void inputGraph(Graph& graph, int edges) {
    cout << "Введите ребра (u, v, вес):\n";
    for (int i = 0; i < edges; ++i) {
        int u, v, weight;
        cin >> u >> v >> weight;
        graph.addEdge(u, v, weight);
    }
}

int main() {
    setlocale(LC_ALL, "ru");
    int vertices, edges;
    cout << "Введите количество вершин: ";
    cin >> vertices;
    cout << "Введите количество ребер: ";
    cin >> edges;

    Graph graph(vertices);
    inputGraph(graph, edges);

    int* distances = new int[vertices];
    int source;
    cout << "Введите исходную вершину для алгоритма Дейкстры: ";
    cin >> source;

    graph.dijkstra(source, distances);
    cout << "Кратчайшие расстояния от вершины " << source << ":\n";
    for (int i = 0; i < vertices; ++i) {
        cout << "Вершина " << i << ": " << distances[i] << "\n";
    }

    cout << "\nМинимальное остовное дерево с использованием алгоритма Прима:\n";
    int** mst_prim = new int* [vertices];
    for (int i = 0; i < vertices; ++i) {
        mst_prim[i] = new int[3];
    }
    graph.prim(mst_prim);
    for (int i = 1; i < vertices; ++i) {
        cout << "Ребро (" << mst_prim[i][0] << ", " << mst_prim[i][1] << ") с весом " << mst_prim[i][2] << "\n";
    }

    cout << "\nМинимальное остовное дерево с использованием алгоритма Крускала:\n";
    int** mst_kruskal = new int* [edges];
    for (int i = 0; i < edges; ++i) {
        mst_kruskal[i] = new int[3];
    }
    int edgeCount = 0;
    graph.kruskal(mst_kruskal, edgeCount);
    for (int i = 0; i < edgeCount; ++i) {
        cout << "Ребро (" << mst_kruskal[i][0] << ", " << mst_kruskal[i][1] << ") с весом " << mst_kruskal[i][2] << "\n";
    }

    delete[] distances;
    for (int i = 0; i < vertices; ++i) {
        delete[] mst_prim[i];
    }
    delete[] mst_prim;
    for (int i = 0; i < edges; ++i) {
        delete[] mst_kruskal[i];
    }
    delete[] mst_kruskal;

    return 0;
}
