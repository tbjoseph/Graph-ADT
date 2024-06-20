/*****************************************
** File:    graph.h
** Project: CSCE 221 Project Spring 2022
** Authors: Timothy Joseph, Patrick Apgar
** Date:    05/7/2022
** Section: 510
** E-mail:  timothy.b.joseph@tamu.edu
**          patrickapgar@tamu.edu
**
** This file contains the header file for Project: Tasks 1 & 2
** This program defines the functions of the Graph ADT
**
***********************************************/

#ifndef GRAPH_H
#define GRAPH_H

#include <list>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <queue>

using std::unordered_map, std::list, std::priority_queue, std::pair, std::make_pair;

class Graph {

struct Node{
    size_t id;
    list<Node*> adjacentNodes;
    bool isKnown = false;
    double cost = INFINITY;
    Node* path;
    // distance variable


    Node(size_t newId){
        id = newId;
    }
};

private:
    // TODO(student): implement graph
    unordered_map<size_t, Node*> vertices;
    unordered_map<size_t, unordered_map<size_t, double>> edges;

public:
    // Task 1

    //-------------------------------------------------------
    // Name: Graph
    // PreCondition:  Graph is not constructed
    // PostCondition: Makes an empty Graph
    //---------------------------------------------------------
    Graph() {}

    //-------------------------------------------------------
    // Name: Graph
    // PreCondition:  Graph is not constructed
    // PostCondition: Constructs a deep copy of a graph
    //---------------------------------------------------------
    Graph(const Graph& other) {
        for(auto pair : other.vertices){
            add_vertex(pair.first); // First should be id
        }

        for(auto [src, map] : other.edges){
            for(auto [dest, value] : map){
                add_edge(src, dest, value);
            }
        }
    }

    //-------------------------------------------------------
    // Name: Graph
    // PreCondition:  Graph is constructed
    // PostCondition: Assigns a deep copy of a graph
    //---------------------------------------------------------
    Graph& operator=(const Graph& other) {
        if(this != &other){

            for(auto pair : other.vertices){
                add_vertex(pair.first);
            }

            for(auto [src, map] : other.edges){
                for(auto [dest, value] : map){
                    add_edge(src, dest, value);
                }
            }
        }

        return *this;
    }

    //-------------------------------------------------------
    // Name: Graph
    // PreCondition:  Graph is constructed (not a null pointer)
    // PostCondition: Destructs a graph (frees all dynamically allocated memory)
    //---------------------------------------------------------
    ~Graph() {
        for(auto pair : vertices){
            delete pair.second;
        }
    }

    //-------------------------------------------------------
    // Name: vertex_count
    // PreCondition:  Graph is constructed, which implies map of verticies is constructed
    // PostCondition: Returns the number of vertices in the graph
    //---------------------------------------------------------
    size_t vertex_count() const {
        return vertices.size();
    }

    //-------------------------------------------------------
    // Name: edge_count
    // PreCondition:  Graph is constructed, which implies map of edges is constructed
    // PostCondition: Returns the number of edges in the graph
    //---------------------------------------------------------
    size_t edge_count() const {
        size_t result = 0;
        
        for(auto pair : edges){
            result += pair.second.size();
        }

        return result;
    }

    //-------------------------------------------------------
    // Name: contains_vertex
    // PreCondition:  Graph is constructed, which implies map of verticies is constructed
    // PostCondition: Return true if the graph contains a vertex with the specified identifier, false otherwise
    //---------------------------------------------------------
    bool contains_vertex(size_t id) const {
        return vertices.count(id) == 1;
    }

    //-------------------------------------------------------
    // Name: contains_edge
    // PreCondition:  Graph is constructed, which implies map of edges is constructed
    // PostCondition: Return true if the graph contains an edge with the specified members (as identifiers), false otherwise
    //---------------------------------------------------------
    bool contains_edge(size_t src, size_t dest) const {

        if(contains_vertex(src) && contains_vertex(dest)){
            if(edges.count(src) == 1){
                return edges.at(src).count(dest) == 1;
            }
        }

        return false;
    }

    //-------------------------------------------------------
    // Name: cost
    // PreCondition:  Graph is constructed, which implies map of edges is constructed
    // PostCondition: Returns the weight of the edge between src and dest, or INFINITY if none exists
    //---------------------------------------------------------
    double cost(size_t src, size_t dest) const {
        // Check for errors
        if(contains_edge(src, dest)){
            return edges.at(src).at(dest);
        }

        return INFINITY;
    }

    //-------------------------------------------------------
    // Name: add_vertex
    // PreCondition:  Vertex to add does not already exist
    // PostCondition: Add a vertex with the specified identifier, return true on success or false otherwise
    //---------------------------------------------------------
    bool add_vertex(size_t id) {
        // The id is the index of the out vector, not inner

        if(contains_vertex(id)) {
            return false;
        }

        Node* node = new Node(id);
        vertices[id] = node;
        return true;
    }

    //-------------------------------------------------------
    // Name: add_edge
    // PreCondition:  Edge to add does not already exist
    // PostCondition: Add a directed edge from src to dest with the specified weight; return true on success, false otherwise
    //---------------------------------------------------------
    bool add_edge(size_t src, size_t dest, double weight=1) {

        if(contains_vertex(src) && contains_vertex(dest) && !contains_edge(src, dest)){
            edges[src][dest] = weight;
            vertices[src]->adjacentNodes.push_back(vertices[dest]);
            return true;
        }

        return false;
    }

    //-------------------------------------------------------
    // Name: remove_vertex
    // PreCondition:  Vertex to remove exists
    // PostCondition: Remove the specified vertex from the graph, including all edges of 
    //                which it is a member; return true on success, false otherwise
    //---------------------------------------------------------
    bool remove_vertex(size_t id) {

        if(!contains_vertex(id)){
            return false;
        }

        // Remove adjacent nodes and edges for all nodes that relate to the soon-to-be deleted vertex
        for(auto [v_id, node] : vertices){
            node->adjacentNodes.remove(vertices[id]);
            remove_edge(v_id, id);
        }

        // Delete edge for current vertex
        edges.erase(id);

        // Delete current vertex
        Node* node = vertices[id];
        vertices.erase(id);
        delete node;

        return true;
    }
    
    //-------------------------------------------------------
    // Name: remove_edge
    // PreCondition:  Edge to remove exists
    // PostCondition: Remove the specified edge from the graph, but do not remove 
    //                the vertices; return true on success, false otherwise. 
    //---------------------------------------------------------
    bool remove_edge(size_t src, size_t dest) {
        if(contains_edge(src, dest)){
            edges[src].erase(dest);
            vertices[src]->adjacentNodes.remove(vertices[dest]);
            return true;
        }

        return false;
    }

    // Task 2

    //-------------------------------------------------------
    // Name: prim
    // PreCondition:  Source vertex exists
    // PostCondition: Computes the minimum spanning tree from the specified source vertex to all other 
    //                vertices in the graph using Prim’s algorithm
    //---------------------------------------------------------
    void prim(size_t source_id) {

        if(!contains_vertex(source_id)){
            return;
        }

        /* 
        
            1: Keep track of all vertices that have been visited/added to spanning tree
                - keep track of these in a priority queue sorted by lowest cost

            2. Spanning tree is empty by default
            3. Choose a random vertex and add to spanning tree. This will be the root node
            4. Add a new vertex such that:
                - vertex is not already in spanning tree
                - vertex is connected to the built spanning tree
                - adding the vertex should not form cycles

            5. Repeat stap 4 until all vertices are added into spanning tree


            Each vertex has an adjacency list, go to pair that matches the source id in 
            vertices table
                - iterate through the node's adjacency list and compare the weight of the edge
                - if the current edge's weight is less than the current minimum weight so far, reassign minimum weight
                    - store that node in the priority queue

        */

        for(auto pair : vertices){
            pair.second->cost = INFINITY;
            pair.second->path = nullptr;
            pair.second->isKnown = false;
        }

        // Set source node cost to 0
        Node* source = vertices.at(source_id);
        source->cost = 0;

        auto compare = [](Node* left, Node* right) {return (left->cost) > (right->cost);};
        priority_queue<Node*, std::vector<Node*>, decltype(compare)> primQueue(compare);
        primQueue.push(vertices[source_id]);

        // Iterate until priority queue is empty
        while(!primQueue.empty()){
            // We need a node
            // We need to know if this node has already been visited
            // We also need the source node's adjacency list

            Node* currentNode = primQueue.top();
            primQueue.pop();

            // if(currentNode->isKnown){
            //     continue;
            // }

            currentNode->isKnown = true;

            // Iterate through node adjacency list
            for(Node* node : currentNode->adjacentNodes){
                // We need weights
                double weight = cost(currentNode->id, node->id);
                
                if(!node->isKnown && weight < node->cost){
                    primQueue.push(node);
                    node->cost = weight;
                    node->path = currentNode;
                }
            }
        }

    }
    
    //-------------------------------------------------------
    // Name: is_path
    // PreCondition:  Prim’s has been run
    // PostCondition: Returns true if there is a path from the Prim-source vertex to the specified destination vertex
    //---------------------------------------------------------
    bool is_path(size_t id) const {

        if(!contains_vertex(id)){
            return false;
        }

        Node* node = vertices.at(id);
        if(node->cost != INFINITY){
            return true;
        }

        return false;
    }

    //-------------------------------------------------------
    // Name: print_path_helper
    // PreCondition:  Prim’s has been run
    // PostCondition: Recursively prints nodes. Helper functon for print_path
    //---------------------------------------------------------
    void print_path_helper(size_t id, std::ostream& os=std::cout) const {
        Node* node = vertices.at(id);
        if(node->path == nullptr){
            // source node
            os << node->id;
            return;
        } else {
            print_path_helper(node->path->id, os);
            os << " --> " << node->id;
        }
    }
    
    //-------------------------------------------------------
    // Name: print_path
    // PreCondition:  Prim’s has been run
    // PostCondition: Pretty prints the minimum spanning path from the Prim source vertex to the specified destination vertex in a 
    //                “ --> “- separated list from source to destination, or prints “<no path>\n” if the vertex is unreachable.
    //---------------------------------------------------------
    void print_path(size_t dest_id, std::ostream& os=std::cout) const {
        if(is_path(dest_id)){
            print_path_helper(dest_id, os);
            os << std::endl;
        } else {
            os << "<no path>\n";
        }
    }

    // Task 3
    
    //-------------------------------------------------------
    // Name: dijkstra
    // PreCondition:  Source vertex exists
    // PostCondition: computes the shortest path from the specified source vertex to
    //                all other vertices in the graph using Dijkstra’s algorithm
    //---------------------------------------------------------
    void dijkstra(size_t source_id) {
        // How is dijkstra's diff from prim's

        if(!contains_vertex(source_id)){
            return;
        }

        /* 
        
        1.  for each vertex
                set the distance to INF
                set visited to false
                set path to nullptr
            
        2.  set distance of the source to 0
        
        3. while there is some unknown # of vertices (!queue.empty())
                v = unvisited vertex w minimum distance (aka currentNode)
                v.isKnown = true
            
                for each unkown vertex (for node in currentNode->adjacentNodes)
                    cvn = distance from v to node

                    if((currentNode->distance + cvn) < node->distance)
                        node->cost = currentNode->distance + cvn
                        node->path = currentNode

        */

       for(auto pair : vertices){
            pair.second->cost = INFINITY;
            pair.second->path = nullptr;
            pair.second->isKnown = false;
        }

        // Set source node cost to 0
        Node* source = vertices.at(source_id);
        source->cost = 0;

        auto compare = [](Node* left, Node* right) {return (left->cost) > (right->cost);};
        priority_queue<Node*, std::vector<Node*>, decltype(compare)> dijkstraQueue(compare);
        dijkstraQueue.push(vertices[source_id]);

        // Iterate until priority queue is empty
        while(!dijkstraQueue.empty()){
            Node* currentNode = dijkstraQueue.top();
            dijkstraQueue.pop();
            currentNode->isKnown = true;

            // Iterate through node adjacency list
            for(Node* node : currentNode->adjacentNodes){
                double weight = cost(currentNode->id, node->id);
 
                if((currentNode->cost + weight) < node->cost){
                    dijkstraQueue.push(node);
                    node->cost = currentNode->cost + weight;
                    node->path = currentNode;
                }
            }
        }
    }

    //-------------------------------------------------------
    // Name: distance
    // PreCondition:  Dijkstra’s has been run
    // PostCondition: Returns the cost of the shortest path from the Dijkstra-source vertex to the specified 
    //                destination vertex, or INFINITY if the vertex or path does not exist.
    //---------------------------------------------------------
    double distance(size_t id) const {

        if(contains_vertex(id)){
            Node* result = vertices.at(id);
            return result->cost;
        }

        return INFINITY;
    }

    //-------------------------------------------------------
    // Name: shortest_path_helper
    // PreCondition:  Dijkstra’s has been run
    // PostCondition: Recursively prints nodes. Helper functon for print_shortest_path
    //---------------------------------------------------------
    void shortest_path_helper(size_t id, std::ostream& os=std::cout) const {
        Node* node = vertices.at(id);
        if(node->path == nullptr){
            // source node
            os << node->id;
            return;
        } else {
            print_path_helper(node->path->id, os);
            os << " --> " << node->id;
        }
    }

    //-------------------------------------------------------
    // Name: print_shortest_path
    // PreCondition:  Dijkstra’s has been run
    // PostCondition: Pretty prints the shortest path from the Dijkstra source vertex to the specified destination vertex in a 
    //                “ --> “- separated list with “ distance: #####” at the end, where <distance> is the minimum cost of a path
    //                from source to destination, or prints “<no path>\n” if the vertex is unreachable.
    //---------------------------------------------------------
    void print_shortest_path(size_t dest_id, std::ostream& os=std::cout) const {
        if(is_path(dest_id)){
            
            shortest_path_helper(dest_id, os);
            os << " distance: " << distance(dest_id) << std::endl;
        } else {
            os << "<no path>\n";
        }
    }
};

#endif  // GRAPH_H