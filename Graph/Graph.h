#pragma once
#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <iostream>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

#define UNDISCOVERED -1
#define DISCOVERED 0
#define VISITED 1
#define UNFIND -404
#define NONE 0

template <typename Tv> struct Vertex
{
	Tv data;
	int status;
	int degree;
	Vertex(Tv Data)
	{
		status = UNDISCOVERED;
		degree = 0;
		data = Data;
	}
};

template <typename Te> struct Edge
{
	Te data;
	int weight;
	int begin;
	int end;
	Edge(Te Data,int Weight, int Begin, int End)
	{
		data = Data;
		weight = Weight;
		begin = Begin;
		end = End;
	}
};

template <typename Tv, typename Te> class Graph
{
private:
	vector<Vertex<Tv>> vertex;
	vector<Edge<Te>> edge;
	vector<vector<int>> AD_matrix;
	vector<vector<int>> ED_matrix;
	void Reset_VertexStatus();
	vector<int> bfs_SingleSerch(int start);

public:
	void push_v(Tv data);
	void push_e(Te data, int weight, int begin, int end);
	void pop_e(int begin, int end);
	vector<int> operator[](int num);
	void Build_Matrix();
	void Print_ADmatrix();
	void Build_EDmatrix();
	void Print_EDmatrix();
	int find_v(Tv goal);
	vector<vector<int>> BFS_Segment();
	vector<vector<bool>> Warshall();
	bool DFS_Search(int start, int goal);
	void Prim();
	void Dijkstra();
};

//��Ӷ���
template <typename Tv, typename Te>
void Graph<Tv, Te>::push_v(Tv data)
{
	Vertex<Tv> _buffer(data);
	vertex.push_back(_buffer);
}

//��ӱ�
template <typename Tv, typename Te>
void Graph<Tv, Te>::push_e(Te data, int weight, int begin, int end)
{
	if (begin < vertex.size() && end < vertex.size() && weight != NONE)
	{
		Edge<Te> _buffer(data, weight, begin, end);
		edge.push_back(_buffer);
		vertex[begin].degree++;
		vertex[end].degree++;
	}
	else cout << "Error: can't add this edge E(" << begin << "," << end << ")" << endl;
}

//ɾ������(begin,end)�ı�
template <typename Tv, typename Te>
void Graph<Tv, Te>::pop_e(int begin, int end)
{
	if (begin < AD_matrix.size() && end < AD_matrix[begin].size())AD_matrix[begin][end] = NONE;
	int e;
	int e_now = 0;
	int old_size = edge.size();
	bool count = 0;
	for (e = 0; e < old_size; e++)
	{
		if (edge[e_now].begin == begin && edge[e_now].end == end) 
		{
			count = 1; 
			edge.erase(std::begin(edge) + e_now);
			e_now--;
		}
		e_now++;
	}
	if(count==0) cout << "Error: can't delete edge E(" << begin << "," << end << ")" << endl;
}

//��ͨ��Graph[][]���������ڽӾ���Ԫ�أ����ǲ����޸ģ�
template <typename Tv, typename Te>
vector<int> Graph<Tv, Te>::operator[](int num)
{
	vector<int> result(0);
	if (num >= AD_matrix.size()) 
	{
		cout << "Error: vector out of range! Check if you have already build the matrix."; return result;
	}
	result = AD_matrix[num];
	return result;
}

//�����ڽӾ���
template <typename Tv, typename Te>
void Graph<Tv, Te>::Build_Matrix()
{
	int vertex_size = vertex.size();
	vector<int> buffer_vector(vertex_size,NONE);
	vector<vector<int>> buffer_matrix(vertex_size, buffer_vector);
	for (int e = 0; e < edge.size(); e++)
	{
		if(edge[e].begin< vertex_size&& edge[e].end< vertex_size)
		buffer_matrix[edge[e].begin][edge[e].end] = edge[e].weight;
		else cout << "Error: no vertex in this edge -- E(" << edge[e].begin << "," << edge[e].end << ")" << endl; 
	}
	AD_matrix = buffer_matrix;
}

//��ӡ�ڽӾ���
template <typename Tv, typename Te>
void Graph<Tv, Te>::Print_ADmatrix()
{
	if (AD_matrix.size() == 0) { cout << "The AD Matrix is empty!" << endl; return; }
	for (int i = 0; i < AD_matrix.size(); i++)
	{
		for (int j = 0; j < AD_matrix[i].size(); j++)
		{
			cout << AD_matrix[i][j] << " ";
		}
		cout << endl;
	}
}

//������������
template <typename Tv, typename Te>
void Graph<Tv, Te>::Build_EDmatrix()
{
	vector<int> buffer_vector(edge.size());
	vector<vector<int>> buffer_matrix(vertex.size(), buffer_vector);
	for (int i = 0; i < edge.size(); i++)
	{
		buffer_matrix[edge[i].begin][i] = 1;
		buffer_matrix[edge[i].end][i] = -1;
	}
	ED_matrix = buffer_matrix;
}

//��ӡ��������
template <typename Tv, typename Te>
void Graph<Tv, Te>::Print_EDmatrix()
{
	if (ED_matrix.size() == 0) { cout << "The ED Matrix is empty!" << endl; return; }
	for (int i = 0; i < ED_matrix.size(); i++)
	{
		for (int j = 0; j < ED_matrix[i].size(); j++)
		{
			cout << ED_matrix[i][j] << " ";
		}
		cout << endl;
	}
}

//Ѱ��ֵΪgoal�Ķ��㣬���ض�����
template <typename Tv, typename Te>
int Graph<Tv, Te>::find_v(Tv goal)
{
	for (int v = 0; v < vertex.size(); v++)
	{
		if (vertex[v].data == goal)return v;
	}
	cout << "Error: can't find the vertex" << endl;
	return UNFIND;
}

//���ö������״̬
template <typename Tv, typename Te>
void Graph<Tv, Te>::Reset_VertexStatus()
{
	for (int v = 0; v < vertex.size(); v++)
	{
		vertex[v].status = UNDISCOVERED;
	}
}

//����DFS�����㷨�ָ��ͨ��֧
template <typename Tv, typename Te>
vector<vector<int>> Graph<Tv, Te>::BFS_Segment()
{
	Reset_VertexStatus();
	if(AD_matrix.size()!=vertex.size())Build_Matrix();

	vector<int> result;
	vector<vector<int>> final_result;
	for (int v = 0; v < vertex.size(); v++)
	{
		if (vertex[v].status == UNDISCOVERED)
		{
			result.clear();
			result = bfs_SingleSerch(v); //һ������ͨ��֧
			final_result.push_back(result);
		}
	}
	return final_result;
}

//DFS����ͨ��֧����
template <typename Tv, typename Te>
vector<int> Graph<Tv, Te>::bfs_SingleSerch(int start)
{
	vector<int> result;
	queue<int> waitinglist;

	waitinglist.push(start);
	vertex[start].status = DISCOVERED;
	while (!waitinglist.empty())
	{
		int buffer = waitinglist.front();
		waitinglist.pop();
		for (int v = 0; v < vertex.size(); v++)
		{
			if ((AD_matrix[v][buffer] != NONE || AD_matrix[buffer][v] != NONE) && vertex[v].status == UNDISCOVERED)
			{
				waitinglist.push(v);
				vertex[v].status = DISCOVERED;
			}
		}
		result.push_back(buffer);
		vertex[buffer].status = VISITED;
	}
	return result;
}

//����Warshall�㷨�ĵ�·������㣬���ص�·����
template <typename Tv, typename Te>
vector<vector<bool>> Graph<Tv, Te>::Warshall()
{
	int vertex_size = vertex.size();
	vector<bool> buffer_vector(vertex_size,NONE);
	vector<vector<bool>> Roadm(vertex_size,buffer_vector);
	for (int e = 0; e < edge.size(); e++)
	{
		if (edge[e].begin < vertex_size&& edge[e].end < vertex_size)
			Roadm[edge[e].begin][edge[e].end] = 1; 
		else cout << "Error: no vertex in this edge -- E(" << edge[e].begin << "," << edge[e].end << ")" << endl;
	}
	
	for(int i=0;i < vertex_size;i++)
		for(int j=0;j < vertex_size;j++)
			for (int k = 0; k < vertex_size; k++)
				Roadm[j][k] = Roadm[j][k] || (Roadm[j][i] && Roadm[i][k]);
			
	return Roadm;
}

//����DFS�����㷨�ж�������Ƿ���ڵ�·
template <typename Tv, typename Te>
bool Graph<Tv, Te>::DFS_Search(int start, int goal)
{
	Reset_VertexStatus();
	if (AD_matrix.size() != vertex.size())Build_Matrix();
	bool count = 0;

	stack<int> waitinglist;
	waitinglist.push(start);
	vertex[start].status = DISCOVERED;
	while (!waitinglist.empty())
	{
		int top = waitinglist.top();
		if (top == goal) { count = 1; break;}
		waitinglist.pop();
		for (int v = 0; v < vertex.size(); v++)
		{
			if (AD_matrix[top][v] != NONE && vertex[v].status == UNDISCOVERED)
			{
				waitinglist.push(v);
				vertex[v].status = DISCOVERED;
			}
		}
		vertex[top].status = VISITED;
	}

	return count;
}

#endif //_GRAPH_H_

//��������ͼ�����½������...��
//ɾ���ߡ��ڵ���� ����ɾ���������
//���Ӳ�ͬͼʾ֮���ת�� ���������AD��EDת��
//DFS BFS�㷨 ���������BFS�ĵ���ͨ�ָDFS��·����
//Prim Dijkstra�㷨
//���Կ���дһд����[]��������� Ŀǰ�������޸�
//���������б��п����ظ������⣨�����ᵼ�¾���ͱ��б���ܲ�һ�£�����ɾ��ʱ����a��b��֮�������ȫ��ɾȥ