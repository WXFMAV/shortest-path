#include "G_Graph.h"
#include<iostream>
#include<algorithm>
#include<glog/logging.h>
using namespace std;

namespace G_Graph{
//创建邻接表的节点
struct AdjListNode* newAdjListNode(int dest, weight_type weight) {
	struct AdjListNode* newNode = (struct AdjListNode*) malloc(
			sizeof(struct AdjListNode));
	newNode->dest = dest;
	newNode->weight = weight;
	newNode->next = NULL;
	return newNode;
}

//创建一个图，包含V的顶点
struct Graph* createGraph(int V) {
	struct Graph* graph = (struct Graph*) malloc(sizeof(struct Graph));
	graph->V = V;

	graph->array = (struct AdjList*) malloc(V * sizeof(struct AdjList));
	graph->array_in = (struct AdjList*) malloc(V * sizeof(struct AdjList));

	for (int i = 0; i < V; ++i){
		graph->array[i].head = NULL;
		graph->array_in[i].head = NULL;
	}

	return graph;
}

int freeAdj(struct AdjListNode *node){
	if(node == NULL) return 0;
	else{
		freeAdj(node->next);
		free(node);
	}
	return 0;
}
int  freeGraph(struct Graph* &graph){

	for(int i = 0; i< graph->V; i++){
		freeAdj(graph->array[i].head);
		freeAdj(graph->array_in[i].head);
	}
	free(graph->array);
	free(graph->array_in);
	free(graph);
	graph = NULL;
	return 0;
}
// 添加一个边(无向图)
void addEdge(struct Graph* graph, int src, int dest, weight_type weight) {


	struct AdjListNode* newNode = newAdjListNode(dest, weight);
	newNode->next = graph->array[src].head;
	graph->array[src].head = newNode;


	newNode = newAdjListNode(src, weight);
	newNode->next = graph->array_in[dest].head;
	graph->array_in[dest].head = newNode;

}

// 最小堆节点
struct MinHeapNode {
	int v;  //下标
	weight_type dist; //距离
};

// 最小堆
struct MinHeap {
	int size;
	int capacity;
	int *pos;     // pos[i]表示顶点i所在的下标
	struct MinHeapNode **array;
};

// 创建一个最小堆节点
struct MinHeapNode* newMinHeapNode(int v, weight_type dist) {
	struct MinHeapNode* minHeapNode = (struct MinHeapNode*) malloc(
			sizeof(struct MinHeapNode));
	minHeapNode->v = v;
	minHeapNode->dist = dist;
	return minHeapNode;
}

// A utility function to create a Min Heap
struct MinHeap* createMinHeap(int capacity) {
	struct MinHeap* minHeap = (struct MinHeap*) malloc(sizeof(struct MinHeap));
	minHeap->pos = (int *) malloc(capacity * sizeof(int));
	minHeap->size = 0;
	minHeap->capacity = capacity;
	minHeap->array = (struct MinHeapNode**) malloc(
			capacity * sizeof(struct MinHeapNode*));
	return minHeap;
}

int freeMinHeap(struct MinHeap * &heap) {
	free(heap->pos);
	free(heap->array);
	free(heap);
	heap= NULL;
	return 0;
}

// 交换两个最小堆的节点
void swapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b) {
	struct MinHeapNode* t = *a;
	*a = *b;
	*b = t;
}

//在位置 idx 调整堆
void minHeapify(struct MinHeap* minHeap, int idx) {
	int smallest, left, right;
	smallest = idx;
	left = 2 * idx + 1;
	right = 2 * idx + 2;

	if (left < minHeap->size
			&& minHeap->array[left]->dist < minHeap->array[smallest]->dist)
		smallest = left;

	if (right < minHeap->size
			&& minHeap->array[right]->dist < minHeap->array[smallest]->dist)
		smallest = right;

	if (smallest != idx) {
		// 需要交换的节点
		MinHeapNode *smallestNode = minHeap->array[smallest];
		MinHeapNode *idxNode = minHeap->array[idx];

		//交换下标
		minHeap->pos[smallestNode->v] = idx;
		minHeap->pos[idxNode->v] = smallest;

		//交换节点
		swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]);

		minHeapify(minHeap, smallest);
	}
}

// 推是否为空
int isEmpty(struct MinHeap* minHeap) {
	return minHeap->size == 0;
}

// 弹出堆顶的节点(即最小的节点)
struct MinHeapNode* extractMin(struct MinHeap* minHeap) {
	if (isEmpty(minHeap))
		return NULL;

	struct MinHeapNode* root = minHeap->array[0];

	struct MinHeapNode* lastNode = minHeap->array[minHeap->size - 1];
	minHeap->array[0] = lastNode;

	// 更新下标
	minHeap->pos[root->v] = minHeap->size - 1;
	minHeap->pos[lastNode->v] = 0;

	// 记得减少堆的大小
	--minHeap->size;
	minHeapify(minHeap, 0);

	return root;
}

// 当节点v的距离更新后(变小了)调整堆
void decreaseKey(struct MinHeap* minHeap, int v, weight_type dist) {
	//获取节点 v 在 堆中的下标
	int i = minHeap->pos[v];

	minHeap->array[i]->dist = dist;

	// 因为是变小了，自下向上调整堆即可。 O(Logn)
	while (i && cmp_weight(minHeap->array[i]->dist ,  minHeap->array[(i - 1) / 2]->dist) == -1) {
		minHeap->pos[minHeap->array[i]->v] = (i - 1) / 2;
		minHeap->pos[minHeap->array[(i - 1) / 2]->v] = i;
		swapMinHeapNode(&minHeap->array[i], &minHeap->array[(i - 1) / 2]);

		i = (i - 1) / 2;
	}
}

// 判断节点v是否在堆中
bool isInMinHeap(struct MinHeap *minHeap, int v) {
	if (minHeap->pos[v] < minHeap->size)
		return true;
	return false;
}

// 打印结果
void printArr(weight_type dist[], int n) {
	printf("Vertex   Distance from Source\n");
	for (int i = 0; i < n; ++i)
		printf("%d \t\t %.2lf\n", i, dist[i]);
}

int cmpEdge2(int u, int v, weight_type weight, weight_type disu, weight_type disv, weight_type &newdisv)
{

	cout<< " max should not be used in dijkstra  will caused error!";
	cout <<"u v w disu disv "<<u<<" "<<v<<" "<<weight<<" "<<disu<<" "<<disv<<" "<<(max(disu, weight) < disv)<<endl;

	int cmp;
	if( max(disu, weight) < disv){
		newdisv = max(disu, weight);
		cmp = -1;
	}
	else if( max(disu, weight) == disv){
		cmp = 0;
	}
	else
	{
		cmp = 1;
	}
	return cmp;
}
int cmp_weight(const weight_type &w1, const weight_type &w2)
{
	if( fabs(w1 - w2) < 0.0001) return 0;
	else{
		if(w1 < w2) return -1;
		else return 1;
	}
}
int cmpEdge(int u, int v, weight_type weight, weight_type disu, weight_type disv, weight_type &newdisv)
{
	int cmp;
	//cout <<"u v w disu disv "<<u<<" "<<v<<" "<<weight<<" "<<disu<<" "<<disv<<" "<<int(disu + weight < disv)<<endl;
	if(cmp_weight(disu + weight , disv) == -1){
		newdisv = disu +weight;
		cmp = -1;
	}
	else if( cmp_weight(disu + weight, disv) == 0){
		cmp = 0;
	}
	else{
		cmp = 1;
	}

	return cmp;
}

bool dijkstra(struct Graph* graph, int src, int tgt, double value_nosolution_jump, std::vector<int> &path) {
	bool found = false;
	int V = graph->V;
	weight_type dist[V];
	path.clear();

	struct MinHeap* minHeap = createMinHeap(V);

	// 初始化堆包含所有的顶点
	for (int v = 0; v < V; ++v) {
		dist[v] = weight_max;
		minHeap->array[v] = newMinHeapNode(v, dist[v]);
		minHeap->pos[v] = v;
	}

	// 把 源点 src 的距离设置为0，第一个取出的点即为源点
	minHeap->array[src] = newMinHeapNode(src, dist[src]);
	minHeap->pos[src] = src;
	dist[src] = weight_zero;
	decreaseKey(minHeap, src, dist[src]);

	minHeap->size = V;

	// 这个循环中，minHeap包含的是所有未在SPT中的顶点
	while (!isEmpty(minHeap)) {
		// 取得堆顶节点，即最小距离的顶点
		struct MinHeapNode* minHeapNode = extractMin(minHeap);
		int u = minHeapNode->v;
		//cout << u << " "<<dist[u]<<endl;
		if ( u == tgt ) break; //找到目标点，不必再搜索。。

		// 只需要遍历和u相邻的顶点进行更新
		struct AdjListNode* pCrawl = graph->array[u].head;
		while (pCrawl != NULL) {
			int v = pCrawl->dest;
			// 松弛操作，更新距离
			if (isInMinHeap(minHeap, v) && cmp_weight(dist[u], weight_max) != 0){
				weight_type newdis = dist[v];

				int cmp = cmpEdge(u, v, pCrawl->weight, dist[u], dist[v], newdis);

			//	cout << "exp :u-v w cmp "<<u<< ' '<<v<< ' '<<pCrawl->weight<<' '<<cmp<<endl;
				if(cmp == -1){
					dist[v] = newdis;
					//距离更新了之后，要调整最小堆
					decreaseKey(minHeap, v, dist[v]);
				}
			}
			pCrawl = pCrawl->next;
		}
	}

	// 打印
	//printArr(dist, V);

	int u = tgt;
	path.clear();

	cout<<"tracing path"<<endl;
	found = true;
	while(cmp_weight(dist[u] , weight_max) != 0){
		if(dist[u] < value_nosolution_jump){
			path.push_back(u);
		}
		else{
			found = false;
		}
		cout<<u<<" "<<dist[u]<<endl;
		if ( u == src) break;
		else{
			int nextu = u;
			struct AdjListNode* pCrawl = graph->array_in[u].head;
			while (pCrawl != NULL) {
				int v = pCrawl->dest;
				if ( cmp_weight(dist[v], weight_max) != 0){
					weight_type newdis = dist[u];
					int cmp = cmpEdge(v, u, pCrawl->weight, dist[v], dist[u], newdis);
					if( cmp == 0){
						nextu = v;
						break;
					}
				}
				pCrawl = pCrawl->next;
			}
			if(nextu == u){
				//error!
				break;
			}
			dist[u] = weight_max; //decrease this one
			u = nextu;
		}
	}

	cout<<"tracing end"<<endl;
	if( path.size()==0 || path[path.size()-1] != src){
		LOG(ERROR) << "path  not found! size ="<<path.size();
	}
	else{
	}

	freeMinHeap(minHeap);
	return found;

}

}
