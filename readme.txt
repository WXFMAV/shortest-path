iarc1.h:
#define TREE_SEARCH_MAX_DEPTH 2
#define TREE_SEARCH_GAMA 0.5
#define TREE_SEARCH_GAMA_THETA 0.2
#define TREE_SEARCH_GAMA_DISTANCE 0.5
#define TREE_SEARCH_GAMA_INNER 1.0


iarc2.h:
#define TREE_SEARCH_MAX_DEPTH 1
#define TREE_SEARCH_GAMA 0.5
#define TREE_SEARCH_GAMA_THETA 0.2
#define TREE_SEARCH_GAMA_DISTANCE 0.5
#define TREE_SEARCH_GAMA_INNER 1.0

iarc3.h:
#define TREE_SEARCH_MAX_DEPTH 2
#define TREE_SEARCH_GAMA 0.8
#define TREE_SEARCH_GAMA_THETA 0.01
#define TREE_SEARCH_GAMA_DISTANCE 0.5
#define TREE_SEARCH_GAMA_INNER 1.0


iarc4.h://iarc4.cpp : revise 'avoid decision when turnning'
#define TREE_SEARCH_MAX_DEPTH 2
#define TREE_SEARCH_GAMA 0.5
#define TREE_SEARCH_GAMA_THETA 0.2
#define TREE_SEARCH_GAMA_DISTANCE 0.5
#define TREE_SEARCH_GAMA_INNER 1.0

iarc5.h:///iarc5.cpp : revise 'avoid decision when turnning'
#define TREE_SEARCH_MAX_DEPTH 2
#define TREE_SEARCH_GAMA 0.8
#define TREE_SEARCH_GAMA_THETA 0.01
#define TREE_SEARCH_GAMA_DISTANCE 0.01
#define TREE_SEARCH_GAMA_INNER 0.01

simulator6.launch:
targets random start in |x|<5 && |y|<5 && theta free
client4.cpp

simulator7.launch:
targets random start in |x|<5 && |y|<5 && theta free
client5.cpp

simulator8.launch:
noise x3
client4.cpp

simulator9.launch:
noise x3
client5.cpp




