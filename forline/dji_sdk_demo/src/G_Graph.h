/*
 * GGraph.h
 *
 *  Created on: 2017年5月26日
 *      Author: Hualin He
 */

#ifndef DJI_SDK_DEMO_SRC_G_GRAPH_H_
#define DJI_SDK_DEMO_SRC_G_GRAPH_H_

// C / C++实现的 Dijkstra最短路径，图的邻接表表示
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <vector>
#include<iarc_arena_simulator/IARCWaypoint.h>
#include"common.h"

namespace G_Graph{

typedef double weight_type;
static double weight_max = 1e10;
static double weight_zero = 0.0;

// 邻接表的节点
struct AdjListNode {
	int dest;
	weight_type weight;
	struct AdjListNode* next;
};

// 邻接表 结构体
struct AdjList {
	struct AdjListNode *head;  // 指向头节点
};

// 图结构体，V为顶点个数。array为所有的邻接表
struct Graph {
	int V;
	struct AdjList* array;
	struct AdjList* array_in;
};

struct Graph* createGraph(int V);
int  freeGraph(struct Graph* &graph);
void addEdge(struct Graph* graph, int src, int dest, weight_type weight);
bool dijkstra(struct Graph* graph, int src, int tgt, double value_nosolution_jump,std::vector<int> &path, double &total_len);
bool dijkstra_foundmin(struct Graph* graph, int src, int tgt, double value_nosolution_jump, std::vector<int> &path, double &total_len) ;
int cmp_weight(const weight_type &w1, const weight_type &w2);
}


#endif /* DJI_SDK_DEMO_SRC_G_GRAPH_H_ */
