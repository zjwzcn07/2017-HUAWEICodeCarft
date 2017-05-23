#include "min_cost_max_flow.h"


MinCostMaxFlow::MinCostMaxFlow()
{
}

MinCostMaxFlow::~MinCostMaxFlow()
{
}

void MinCostMaxFlow::init()
{
	memset(gHead, -1, sizeof(gHead));
	gEdgeCount = 0;
}

void MinCostMaxFlow::init(int m)
{
	for (int i = 0;i != m;++i)
		gHead[i] = -1;
}

//返回cost和flow
pair<int,int> MinCostMaxFlow::solve(Graph &gp, vector<int>serverId)
{
	init();
	int s = 0;					//超源点
	int t = gp.nodeNum + 1;		//超汇点
	for (auto server : serverId)
	{
		InsertEdge(0, server + 1, gp.nodeMap[server].outSum, 0);		//增加超源点至源点
	}
	for (auto client : gp.clientMap)
	{
		int neighId = client.second.nigId;
		InsertEdge(neighId + 1, t, client.second.demand, 0);				//增加汇点至超汇点
	}
	for (auto link : gp.linkList)
	{
		int out_node = link.fid;
		int in_node = link.sid;
		InsertEdge(out_node + 1, in_node + 1, link.width, link.cost);
		node2edge[make_pair(out_node, in_node)] = gEdgeCount - 2;
	}

	int ans, flow;
	pair<int, int>result = MinCostFlow(s, t);
	ans = result.first;
	flow = result.second;
	min_cost = ans;
	max_flow = flow;
	//cout << "cost = " << ans << " flow = " << flow << endl;
	//PrintFlow(gp);
	return make_pair(ans,flow);
}

void MinCostMaxFlow::PrintFlow(Graph &gp)
{
	printf("-----------------this flow is--------------------\n");
	printf("flow = %d, cost = %d\n", max_flow, min_cost);
	int sum_flow = 0;
	int sum2_flow = 0;
	int pre = 0, nxt = 0, flag = 0;
	for (auto link : gp.linkList)
	{
		int out_node = link.fid;
		int in_node = link.sid;
		int init_flow = link.width;
		int now_flow = gEdges[node2edge[make_pair(out_node, in_node)]].vol;
		cout << "from " << out_node << " to " << in_node << " flow = " << init_flow - now_flow << endl;
		sum_flow += (init_flow - now_flow);
		if (!flag)
		{
			flag = 1;
			pre = init_flow - now_flow;
		}
		else {
			nxt = init_flow - now_flow;
			sum2_flow += abs(pre - nxt);
			flag = 0;
		}
	}
	cout << "sum1 = " << sum_flow << "sum2 = " << sum2_flow << endl;
	printf("------------------------------------------------\n");
}

void MinCostMaxFlow::test()
{
	int n, m;
	while (true)
	{
		cin >> n >> m;
		init();
		int s = 0;
		int t = n + m + 1;
		for (int i = 0;i != n;++i)//源点
		{
			int c;
			cin >> c;
			InsertEdge(0, i + 1, c, 0);
		}
		for (int i = 0;i != m;++i)//汇点
		{
			int c;
			cin >> c;
			InsertEdge(n + i + 1, t, c, 0);
		}
		for (int i = 0;i != n;++i)
		{
			for (int j = 0;j != m;++j)
			{
				int a;
				cin >> a;
				InsertEdge(i + 1, n + j + 1, INF, a);
			}
		}
		int ans, flow;
		pair<int,int>  result = MinCostFlow(s, t);
		ans = result.first;
		flow = result.second;
		cout << "cost = " << ans << "flow = " << flow << endl;
	}
}



void MinCostMaxFlow::InsertEdge(int u, int v, int vol, int cost)
{
	gEdges[gEdgeCount].to = v;
	gEdges[gEdgeCount].vol = vol;
	gEdges[gEdgeCount].init_vol = vol;
	gEdges[gEdgeCount].cost = cost;
	gEdges[gEdgeCount].next = gHead[u];
	gHead[u] = gEdgeCount++;

	gEdges[gEdgeCount].to = u;
	gEdges[gEdgeCount].vol = 0;         //vol为0，表示开始时候，该边的反向不通
	gEdges[gEdgeCount].init_vol = vol;
	gEdges[gEdgeCount].cost = -cost;    //cost 为正向边的cost相反数，这是为了
	gEdges[gEdgeCount].next = gHead[v];
	gHead[v] = gEdgeCount++;
}

//假设图中不存在负权和环,SPFA算法找到最短路径/从源点s到终点t所经过边的cost之和最小的路径
bool MinCostMaxFlow::Spfa(int s, int t) {
	memset(gPre, -1, sizeof(gPre));
	memset(gDist, 0x7F, sizeof(gDist));
	gDist[s] = 0;
	queue<int> Q;
	Q.push(s);
	while (!Q.empty()) {//由于不存在负权和环，因此一定会结束
		int u = Q.front();
		Q.pop();

		for (int e = gHead[u]; e != -1; e = gEdges[e].next) {
			int v = gEdges[e].to;
			if (gEdges[e].vol > 0 && gDist[u] + gEdges[e].cost < gDist[v]) {
				gDist[v] = gDist[u] + gEdges[e].cost;
				gPre[v] = u; //前一个点
				gPath[v] = e;//该点连接的前一个边
				Q.push(v);
			}
		}
	}

	if (gPre[t] == -1)  //若终点t没有设置pre，说明不存在到达终点t的路径
		return false;
	return true;
}

pair<int,int> MinCostMaxFlow::MinCostFlow(int s, int t) {
	int cost = 0;
	int flow = 0;
	while (Spfa(s, t)) {
		int f = INF;
		for (int u = t; u != s; u = gPre[u]) {
			if (gEdges[gPath[u]].vol < f)
				f = gEdges[gPath[u]].vol;
		}
		flow += f;
		cost += gDist[t] * f;
		for (int u = t; u != s; u = gPre[u]) {
			gEdges[gPath[u]].vol -= f;   //正向边容量减少
			gEdges[gPath[u] ^ 1].vol += f; //反向边容量增加
		}
	}
	//cout << "funtion run cost " << cost << " flow = " << flow << endl;
	return make_pair(cost,flow);
}


bool MinCostMaxFlow::Check(Graph &gp)
{
	printf("check start, max flow = %d, min cost = %d, graph need flow = %d\n", max_flow, min_cost, gp.demandSum);

	map<int, bool>have_client;
	have_client.clear();
	int flows = 0;
	bool flag = 1;
	for (auto path : solve_path)
	{
		have_client[path.first.back()] = 1;
		flows += path.second.first;
	}
	for (auto client : gp.clientMap)
	{
		Client cli = client.second;
		if (have_client[client.first] == 0)
		{
			solve_path.push_back(make_pair(vector<int>{cli.nigId, cli.cId}, make_pair(cli.demand, gp.serverCost)));
			printf("gg! %d_%d clinet missing\n", client.first, client.second.nigId);
			flag = 0;
		}
	}
	map<int, int>flow;
	flow.clear();
	int foo = 0;
	for (auto path : solve_path)
	{
		foo += path.second.first;
		flow[path.first.back()] += path.second.first;
	}
	printf("foo = %d\n", foo);
	if (foo < gp.demandSum)
	{
		printf("gg! demandSum can't satisy\n");
	}
	for (auto clinet_flow : flow)
	{
		Client cli = gp.clientMap[clinet_flow.first];
		if (gp.clientMap[clinet_flow.first].demand > clinet_flow.second)
		{
			flag = 0;
			solve_path.push_back(make_pair(vector<int>{cli.nigId, cli.cId}, make_pair(cli.demand, gp.serverCost)));
			printf("gg! %d_%d clinet don't have enough flow: need %d, real %d\n", clinet_flow.first, gp.clientMap[clinet_flow.first].nigId, gp.clientMap[clinet_flow.first].demand, clinet_flow.second);
		}
	}

	for (auto path : solve_path)
	{
		vector<int> pa = path.first;
		pa.pop_back();
		if (pa.size() == 1)
			continue;
		int f = path.second.first;
		for (int i = 0;i < (int)pa.size() - 1;i++)
		{
			int a, b;
			a = pa[i];
			b = pa[i + 1];
			int fo = gEdges[node2edge[make_pair(a, b)]].init_vol;
			if (fo < f)
			{
				flag = 0;
				printf("gg! %d to %d, flow need %d, but only can pass %d\n", a, b, f, fo);
			}
		}
	}
	if (flag)
	{
		printf("this check is ok\n");
	}
	return flag;
}

void MinCostMaxFlow::FindAPath(Graph &gp, vector<int>path, int now, int flow, int sum)
{
	if (gp.nodeMap[now].hasClient && path.size()>1)
	{
		//for (int i = 0;i < (int)path.size() - 1;i++)
		//{
		//	int a = path[i];
		//	int b = path[i + 1];
		//	gEdges[node2edge[make_pair(a, b)]].init_vol -= flow;
		//}

		path.push_back(gp.nodeMap[now].cId);
		solve_path.push_back(make_pair(path,make_pair(flow,sum)));
	}
	else {
		vector<int>links = gp.nodeMap[now].outLids;
		for (auto link : links)
		{
			int in_node = gp.linkList[link].sid;
			int now_flow = gEdges[node2edge[make_pair(now, in_node)]].vol;
			int init_flow = gEdges[node2edge[make_pair(now, in_node)]].init_vol;
			int real_flow = init_flow - now_flow;
			if (real_flow <= 0)
				continue;
			path.push_back(in_node);
			FindAPath(gp, path, in_node, min(flow, real_flow), sum + gp.linkList[link].cost);

			path.pop_back();
		}
	}
}


void MinCostMaxFlow::PrintPath(Graph &gp,vector<int>&servers,char *topo_file)
{
	for (auto server : servers)
	{
		if (gp.nodeMap[server].hasClient)
		{
			int cid = gp.nodeMap[server].cId;
			solve_path.push_back(make_pair(vector<int>{server, cid}, make_pair(gp.clientMap[cid].demand,gp.serverCost)));
		}
		FindAPath(gp, vector<int>{server}, server, INF, 0);
	}

	//FILE *fo = freopen("reslut.txt", "w", stdout);
	//printf("check once\n");
	//if (Check(gp) == 0)
	//{
	//	printf("check again\n");
	//	Check(gp);
	//}

	printf("%d\n", solve_path.size());
	for (auto path : solve_path)
	{
		for (auto node : path.first)
			printf("%d ", node);
		printf("%d", path.second.first);
		printf(" cost = %d\n", path.second.second);
	}


	char line[1000];

	string topoFile;

	sprintf(line, "%d\n", solve_path.size());

	printf("%d\n", solve_path.size());

	topoFile += line;

	for (auto path : solve_path)

	{

		sprintf(line, "\n");

		topoFile += line;

		for (auto node : path.first) {

			sprintf(line, "%d ", node);

			printf("%d ", node);

			topoFile += line;

		}
		sprintf(line, "%d", path.second.first);
		printf("%d", path.second.first);
		printf(" cost = %d\n", path.second.second);
		topoFile += line;
	}
	sprintf(topo_file, "%s", topoFile.c_str());

}



void MinCostMaxFlow::genResult(Graph &G, vector<int>&servers, char *topo_file) {
	map<int, int> serPro, cliDemand;
	vector<vector<int>> routes;
	vector<int> route;

	char line[1000];
	string topoFile;
	int sub = 0;

	for (auto server : servers)
	{
		serPro[server] = 0;
		for (int linkId : G.nodeMap[server].outLids)
		{
			int startNode, endNode;
			startNode = G.linkList[linkId].fid;
			endNode = G.linkList[linkId].sid;
			int now_flow = gEdges[node2edge[make_pair(startNode, endNode)]].vol;
			int init_flow = gEdges[node2edge[make_pair(startNode, endNode)]].init_vol;
			int real_flow = init_flow - now_flow;
			serPro[server] += real_flow;
		}
	}


	for (auto& cit : G.clientMap)
	{
		cliDemand[cit.first] = cit.second.demand;
	}


	for (auto server : servers)
	{
		if (G.nodeMap[server].hasClient)
		{
			cliDemand[G.nodeMap[server].cId] = 0;
			route.clear();
			route.push_back(server);
			route.push_back(G.nodeMap[server].cId);
			route.push_back(G.nodeMap[server].demand);
			routes.push_back(route);
		}
		dispRoute(G, routes, server, serPro, cliDemand);
	}

	map<int, int>serverOut;
	serverOut.clear();
	for (int i = 0;i < routes.size();i++)
	{
		serverOut[routes[i][0]] += routes[i].back();
	}


	sprintf(line, "%d\n", (int)routes.size());
	topoFile += line;
	for (unsigned int i = 0; i < routes.size(); i++)
	{
		sprintf(line, "\n");
		topoFile += line;
		for (unsigned int j = 0; j < routes[i].size(); j++) {
			if (j) topoFile += " ";
			sprintf(line, "%d", routes[i][j]);
			topoFile += line;
		}

		//int need = G.nodeMap[routes[i][0]].outSum;
		int need = serverOut[routes[i][0]];
		int po = G.serverList[0].id, cost = G.serverList[0].cost;
		int ini = po;
		for (auto ser : G.serverList)
		{
			if (ser.maxOut >= need && ser.cost <= cost)
			{
				cost = ser.cost;
				po = ser.id;
			}
		}
		if (ini != po)
		{
			printf("ha~");
			//sub += G.serverList[ini].cost - G.serverList[po].cost;
		}
		sprintf(line, " %d", po);

		//sprintf(line, " %d", G.serverList[0].id);
		topoFile += line;
	}
	//printf("save %d\n", sub);

	sprintf(topo_file, "%s", topoFile.c_str());

}

void MinCostMaxFlow::dispRoute(Graph &G, vector<vector<int>> &routes, int server, map<int, int> &serPro, map<int, int> cliDemand){
	vector<int> route,linkCross;
	while (serPro[server] > 0)
	{

		route.clear();
		linkCross.clear();
		int widthUse = serPro[server];
		int thisNode = server;
		route.push_back(thisNode);
		bool flag=false;
		while (!flag)
		{
			bool findOut=false;
			for (auto linkOut : G.nodeMap[thisNode].outLids)
			{
				int endNode;
				endNode = G.linkList[linkOut].sid;
				int now_flow = gEdges[node2edge[make_pair(thisNode, endNode)]].vol;
				int init_flow = gEdges[node2edge[make_pair(thisNode, endNode)]].init_vol;
				int real_flow = init_flow - now_flow;

				if (real_flow > 0)
				{
					linkCross.push_back(linkOut);
					route.push_back(endNode);
					findOut = true;
					widthUse = min(real_flow, widthUse);
					thisNode = endNode;
					break;
				}
			}
			if (!findOut&&G.nodeMap[thisNode].hasClient&&cliDemand[G.nodeMap[thisNode].cId]>0){
				findOut = true;
				route.push_back(G.nodeMap[thisNode].cId);
				widthUse = min(cliDemand[G.nodeMap[thisNode].cId], widthUse);
				route.push_back(widthUse);
				routes.push_back(route);

				cliDemand[G.nodeMap[thisNode].cId] -= widthUse;
				serPro[server] -= widthUse;
				for (int linkOut : linkCross)
				{
					int startNode, endNode;
					startNode = G.linkList[linkOut].fid;
					endNode = G.linkList[linkOut].sid;
					gEdges[node2edge[make_pair(startNode, endNode)]].vol += widthUse;
				}
				flag = true;
				break;
			}

			if (!findOut){
				break;
			}
		}
	}


}

