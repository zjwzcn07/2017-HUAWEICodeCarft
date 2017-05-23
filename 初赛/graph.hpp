#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <vector>
#include <map>
#include <algorithm>
#include "deploy.h"

using namespace std;

typedef struct Client {
	int				cId;
	int				nigId;
	int				demand;
}Client;
typedef struct Server {
	int				id;
	int				maxOut;
	int				cost;
}Server;

typedef struct Point{
	int	nodeId;
	vector<int>inLids;
	vector<int>outLids;
	bool hasClient, hasServer;
	int	cId, outSum, demand,deployCost;
	int level;
	Point(){
	}

	Point(unsigned int nid, int c){
		nodeId = nid;
		hasClient = false;
		cId = -1;
		outSum = 0;
		demand = 0;
		deployCost = c;
	}

	~Point(){
	}

}Point;


typedef struct Seg {
	int	linkId,fid,sid,cost,width;
}Seg;



typedef struct Graph{
	vector<Seg>		linkList;
	map<int,Point>		nodeMap;
	map<int,Client>	clientMap;
	vector<pair<int, int>>orderNodeList;
	vector<Server>		serverList;

	int		nodeNum;
	int		linkNum;
	int		serverCost;
	int		clientNum;
	int		demandSum;


	void addNode(int nodeId, int c){
		if (nodeMap.find(nodeId) != nodeMap.end())
			return;
		Point n(nodeId, c);
		nodeMap[nodeId] = n;
	}

	void addSeg(int n1, int n2, int w, int c){
		Seg l;
		int lid;
		lid = linkList.size();
		l.cost = c;
		l.width = w;
		l.linkId = lid;
		l.fid = n1;
		l.sid = n2;
		linkList.push_back(l);
		nodeMap[n2].inLids.push_back(lid);
		nodeMap[n1].outLids.push_back(lid);
		nodeMap[n1].outSum += w;
	}


	void createGraph(char *topo[MAX_EDGE_NUM],int line_num){
		demandSum = 0;
		sscanf(topo[0], "%d %d %d", &nodeNum, &linkNum, &clientNum);

		int lineIdx = 2;
		while (topo[lineIdx][0]!='\n' && topo[lineIdx][0] != '\r')
		{
			int a, b, c;
			sscanf(topo[lineIdx++], "%d %d %d", &a, &b, &c);
			Server ser;
			ser.id = a, ser.cost = c, ser.maxOut = b;
			serverList.push_back(ser);
		}

		sort(serverList.begin(), serverList.end(), [](Server a, Server b) {
			return a.maxOut > b.maxOut;
		});
		serverCost = serverList[0].cost;
		lineIdx++;

		for (int i = 0;i < nodeNum;i++, lineIdx++)
		{
			int n1, c;
			sscanf(topo[lineIdx], "%d %d", &n1, &c);
			addNode(n1,c);
		}

		lineIdx++;


		int linkId = 0;
		for (int i = 0; i < linkNum; i++, lineIdx++){
			int n1, n2, w, c;
			sscanf(topo[lineIdx], "%d %d %d %d", &n1, &n2, &w, &c);
			addSeg(n1, n2, w, c);
			addSeg(n2, n1, w, c);
		}


		linkNum = linkNum * 2;
		lineIdx++;
		for (int i = 0; i < clientNum; i++, lineIdx++){
			int cid, neighb, demand;
			sscanf(topo[lineIdx], "%d %d %d", &cid, &neighb, &demand);
			Client client;
			client.cId = cid;
			client.nigId = neighb;
			client.demand = demand;
			demandSum += demand;
			clientMap[cid] = client;
			nodeMap[neighb].hasClient = true;
			nodeMap[neighb].cId = cid;
			nodeMap[neighb].outSum += demand;
			nodeMap[neighb].demand = demand;
		}
	}

}Graph;


#endif