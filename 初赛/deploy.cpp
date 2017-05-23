#include "deploy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>


#include "graph.hpp"
#include "min_cost_max_flow.h"


#include <map>
#include <vector>
#include <algorithm>
#include <ctime>
#include <set>

using namespace std;

int CNT;
const clock_t limit_time = 85 * CLOCKS_PER_SEC;
Graph G;

MinCostMaxFlow MCMF;
clock_t st, ed;


void disp(vector<int>&seq)
{
	printf("\n");
	for (auto v : seq)
	{
		printf("%d ", v);
	}
	printf("\n");
}
vector<int>final_ser;
vector<int>fake_server;
vector<int>cd_ser;
pair<int, int>final_flow;
pair<int, int>fake_cost_flow;
pair<int, int>cand;
map<int, bool>capat;
map<set<int>, bool>vits;

int updata(Graph &G, vector<int>&servers)
{
	cand = MCMF.solve(G, servers);
	CNT += 1;
	//cand.first += (int)servers.size()*G.serverCost;

	for (auto ser : servers)
		cand.first += G.nodeMap[ser].deployCost;

	if (cand.second == G.demandSum)
	{
		if (cand.first < final_flow.first)
		{
			printf("iter=%d\tobj=%d\n", CNT, final_flow.first);
			final_flow = cand;
			final_ser = servers;
			return 1;
		}
	}
	return 0;
}


map<int, bool> trash;
void BF(Graph &G, vector<int>now_seq, int res_need, int now)
{
	clock_t ed = clock();
	if (ed - st > limit_time)
		return;

	if (now >= G.nodeNum || res_need == 0)
	{
		updata(G, now_seq);
	}
}

void DFS2(Graph &G, int now, int left_num, int max_num, vector<int>&cand, vector<int>out)
{
	clock_t ed = clock();
	if (ed - st > limit_time)
		return;

	if ((int)out.size() == left_num || now >= (int)cand.size())
	{
		trash.clear();
		for (auto x : out)
			trash[x] = 1;
		BF(G, out, 0, 0);
	}
	else {
		out.push_back(cand[now]);
		DFS2(G, now + 1, left_num, max_num, cand, out);

		out.pop_back();
		DFS2(G, now + 1, left_num, max_num, cand, out);
	}
}


void BruteForce(Graph &G, vector<int>now_seq, int cnt)
{
	clock_t ed = clock();
	if (ed - st > limit_time)
		return;
	if (now_seq.empty())
		return;

	vector<int>tmp_seq,iter_seq;
	iter_seq.clear();
	for (int i = 0;i != now_seq.size();i++)
	{
		tmp_seq = now_seq;
		tmp_seq.erase(tmp_seq.begin() + i);
		int flag = updata(G, tmp_seq);
		if (flag)
		{
			iter_seq = tmp_seq;
		}
	}
	if ( iter_seq.size()!=0 && iter_seq.size() < now_seq.size())
	{
		now_seq = iter_seq;
		BruteForce(G, now_seq, cnt + 1);
	}
}

void easy_print(Graph &G, char *topo_file)
{
	char line[1000];
	string topoFile;
	topoFile += line;
	for (auto client : G.clientMap)
	{
		sprintf(line, "\n");
		topoFile += line;
		sprintf(line, "%d %d %d %d\n", client.second.nigId, client.second.nigId, client.second.demand, G.serverList[0].id);
		topoFile += line;
	}
	sprintf(topo_file, "%s", topoFile.c_str());
}


void solve(char * topo[MAX_EDGE_NUM], int line_num, char *topo_file){
	G.createGraph(topo, line_num);


	if (line_num > 1000000)
	{
		easy_print(G, topo_file);
		return;
	}

	st = clock();
	map<int, bool>cap_ser;cap_ser.clear();
	for (auto client : G.clientMap)
		final_ser.push_back(client.second.nigId),cap_ser[client.second.nigId]=1;
	for (auto node : G.nodeMap)
	{
		if (node.second.deployCost == 500 && cap_ser[node.second.nodeId] == 0)
		{
			final_ser.push_back(node.second.nodeId);
		}
	}

	sort(final_ser.begin(), final_ser.end(), [](int a, int b) {
		return G.nodeMap[a].outSum*G.nodeMap[b].deployCost> G.nodeMap[b].outSum*G.nodeMap[a].deployCost;
	});

	//final_flow.first = G.clientNum*G.serverCost;
	final_flow.first = 0;
	for (auto ser : final_ser)
		final_flow.first += G.nodeMap[ser].deployCost;

	final_flow.second = G.demandSum;
	
	disp(final_ser);
	//updata(G, final_ser);

	//BruteForce(G, final_ser, 0);

	int max_num = final_ser.size()-1;

	for (int i = max_num;i > 0;i--)
	{
		DFS2(G, 0, i, max_num, final_ser, cd_ser);
	}

	disp(final_ser);
	MCMF.solve(G, final_ser);
	MCMF.genResult(G, final_ser, topo_file);
	topo_file = (char*)"10\n";
}




void deploy_server(char * topo[MAX_EDGE_NUM], int line_num, char * filename){

	char  topo_file[MAX_EDGE_NUM];

	printf("\n\n");
	CNT=0;
		
	solve(topo, line_num, topo_file);
	
	printf("\n\n");

	write_result(topo_file, filename);

}
