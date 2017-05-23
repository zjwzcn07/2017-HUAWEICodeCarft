#ifndef MINCOSTMAXFLOW
#define MINCOSTMAXFLOW
#include <cstring>
#include <queue>
#include <iostream>
#include <map>
using namespace std;

#include "graph.hpp"

#define INF 0x3f3f3f3f
struct Edge
{
	int to;
	int vol;
	int cost;
	int next;
	int init_vol;
};
const int MAX_NODE = 10005;
const int MAX_EDGE = MAX_NODE*2*100 + 5;
class MinCostMaxFlow
{
public:
	MinCostMaxFlow();
	~MinCostMaxFlow();

	Edge	gEdges[MAX_EDGE];

	int		gHead[MAX_NODE];
	int		gPre[MAX_NODE];
	int		gPath[MAX_NODE];
	int		gDist[MAX_NODE];
	int		gEdgeCount;
	int		min_cost;
	int		max_flow;
	map<pair<int, int>, int> node2edge;
	vector<pair<vector<int>,pair<int,int>>> solve_path;

	void InsertEdge(int u, int v, int vol, int cost);
	bool Spfa(int s, int t);
	pair<int,int> MinCostFlow(int s, int t);
	void init();
	void init(int m);
	void test();
	pair<int, int> solve(Graph &gp, vector<int>serverId);
	void PrintFlow(Graph &gp);
	void FindAPath(Graph &gp, vector<int>path, int now, int flow, int sum);
	void PrintPath(Graph &gp, vector<int>&servers, char *topo_file);
	bool Check(Graph &gp);
	void genResult(Graph &G, vector<int>&servers, char *topo_file);
	void dispRoute(Graph &G, vector<vector<int>> &routes, int server, map<int, int> &serPro,map<int, int> cliDemand);
private:

};


#endif