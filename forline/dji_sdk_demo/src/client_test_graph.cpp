#include "G_Graph.h"
#include<iostream>
#include <ros/ros.h>

using namespace G_Graph;
using namespace std;

// 测试
int Graph_main() {
	// 创建上一讲：http://www.acmerblog.com/dijkstra-shortest-path-algorithm-5876.html 例子中的图
	int V = 9;
	struct Graph* graph = createGraph(V);
	addEdge(graph, 0, 1, 4);
	addEdge(graph, 0, 7, 8);
	addEdge(graph, 1, 2, 8);
	addEdge(graph, 1, 7, 11);
	addEdge(graph, 2, 3, 7);
	addEdge(graph, 2, 8, 2);
	addEdge(graph, 2, 5, 4);
	addEdge(graph, 3, 4, 9);
	addEdge(graph, 3, 5, 14);
	addEdge(graph, 4, 5, 10);
	addEdge(graph, 5, 6, 2);
	addEdge(graph, 6, 7, 1);
	addEdge(graph, 6, 8, 6);
	addEdge(graph, 7, 8, 7);

	std::vector<int> path;
	double total_len = 0.0;
	uint32_t t1,t2;
	t1=arena_time_now();
	dijkstra(graph, 0, 4, weight_max, path, total_len);
	t2=arena_time_now();
	cout<<t2-t1<<" ";
	path.clear();
	t1=arena_time_now();
	dijkstra_foundmin(graph, 0, 4, weight_max, path, total_len);
	t2=arena_time_now();
	cout<<t2-t1<<endl;

	cout<<"path size = "<<path.size()<<endl;

	for(int k = path.size()-1; k >= 0 ; k--){
		cout<< path[k]<< " ";
	}
	cout<<endl;

	freeGraph(graph);

	return 0;

}

int Graph_main2()
{
	uint32_t t1= arena_time_now();
	cout<<"t1 = "<<t1<<endl;

	int V = 1000;
	struct Graph* graph = createGraph(V);
	//addEdge(graph, 0, 1, 4);
	srand(1);

	for(int  u = 0; u< V; u++)
	{
		for(int v = u+1; v< V; v++){
			double w = random() % 100;
			addEdge(graph, u, v, w);
			addEdge(graph, v, u, w);
		}
	}

	std::vector<int> path;
	double total_len = 0.0;

	dijkstra(graph, 0, V-1, weight_max,  path, total_len);
	uint32_t t2 = arena_time_now();

	cout<<"t2 = "<<t2<<" t2-t1="<<t2-t1<<endl;

	cout<<"path size = "<<path.size()<<endl;

	for(int k = path.size()-1; k >= 0 ; k--){
		cout<< path[k]<< " ";
	}

	cout<<endl;

}

int Graph_main3()
{
	FILE *fp=fopen(PARAM::file_name_timecost.c_str(),"w");
	uint32_t t1= arena_time_now();
	uint32_t t2=0;

	cout<<"t1 = "<<t1<<endl;

	int V = 3000;
	int m = 3000;
	int weightlen = 1000;

	//for(m = 3000; m<30000; m=((double)m*1.1)){
	for(m = 3000; m<30000; m=m+50){


	/*int V = 1000;
	int m = 1000000;
	int weightlen = 100;

	for(V = 1000; V<10000; V=V+500){*/
		cout<<V<<" "<<m<<" ";
		fprintf(fp,"%d %d ",V,m);
	srand(1);
	struct Graph* graph = createGraph(V);
	for (int k = 0; k<m-V+1; k++)
	{
		int u = random() % V;
		int v = random() % V;
		if(u != v){
			double w = random() % weightlen+1;
			addEdge(graph,u,v,w);
		}
		else{
			k--;
		}
	}
	std::vector<int> nodes;
	nodes.clear();
	for(int k=0; k<V; k++) nodes.push_back(k);
	int last = 0;
	nodes.erase(nodes.begin() + 0);
	for(int k=nodes.size(); k>0; k--)
	{
		int now = random() % nodes.size();
		double w = random() % weightlen+1;
		addEdge(graph,last,nodes[now],w);
		last = nodes[now];
		nodes.erase(nodes.begin() + now);
	}

	std::vector<int> path;
	double total_len = 0.0;

	t1=arena_time_now();
	dijkstra(graph, 0, V-1, weight_max,  path, total_len);
	 t2 = arena_time_now();
	cout<<t2-t1<<" "<<path.size()<<" ";
	fprintf(fp,"%d %d ",t2-t1, path.size());
	path.clear();
	total_len = 0.0;
	t1=arena_time_now();
	dijkstra_foundmin(graph, 0, V-1, weight_max,  path, total_len);
	 t2 = arena_time_now();
	cout<<t2-t1<<" "<<path.size()<<endl;
	fprintf(fp,"%d %d\n",t2-t1, path.size());
	freeGraph(graph);

	}

	fclose(fp);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;

    arena_set_startnow();
	Graph_main3();
	//Graph_main2();
	return 0;
}
