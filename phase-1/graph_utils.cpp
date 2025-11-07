// basic graph functions that are common to all graphs.

#include "Graph.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <queue>
#include <cstring>
#include <limits>


// constructor
Graph::Graph(int n) : numNodes(n) {
    nodes.resize(n);
    adj.resize(n);
}

// destructor
Graph::~Graph() {
    for (auto& edgeList : adj) {
        edgeList.clear(); 
    }
    adj.clear();
    nodes.clear();
    edges.clear();
}

void Graph::addNode(int id, double lat, double lon, const std::vector<std::string>& poisList){

    Node a=nodes[id];

    a.id = id;
    a.lat=lat;
    a.lon=lon;

    a.pois.clear();
    for(auto x:poisList){
        a.pois.push_back(x);
    }

}

void Graph::addEdge(int id, int u, int v, double length, double avg_time,bool oneWay,
                     const std::string &road_type,const std::vector<double>& speed_profile)
{

    Edge e{ id, u, v, length, avg_time, oneWay, speed_profile, road_type, true };

    edges[id]=e;
    adj[u].push_back(&edges[id]);

    // if its both way add the edge in other vertice aswell
    if (!oneWay) {
        adj[v].push_back(&edges[id]);
    }
}

bool Graph::removeEdge(int edge_id){
    // if it is inactive or if the edge is not there return false
    if(!edges[edge_id].active || edges.find(edge_id) == edges.end()) return false;

    edges[edge_id].active=false;

    return true;
}


bool Graph::modifyEdge(int edge_id, const std::vector<double>& new_speed_profile,
                       double new_length, double new_avg_time)
{
    // if there is no such edge or if the patch is empty then returns false
    if(edges.find(edge_id) == edges.end()) return false;

    if(edges[edge_id].active){
       if(new_length == -1) return true;
    }
    if(!edges[edge_id].active){
        edges[edge_id].active=true;
    }
    
    if(new_length >= 0){
            edges[edge_id].length=new_length;
    }
    if(new_avg_time >= 0){
        edges[edge_id].average_time=new_avg_time;
    }

}