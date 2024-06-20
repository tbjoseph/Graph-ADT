/*****************************************
** File:    graph.cpp
** Project: CSCE 221 Project Spring 2022
** Authors: Timothy Joseph, Patrick Apgar
** Date:    05/7/2022
** Section: 510
** E-mail:  timothy.b.joseph@tamu.edu
**          patrickapgar@tamu.edu
**
** This file contains the main driver program for Project: Tasks 1 & 2.
** This program tests the various functions of graph.h
** and ensures that the Graph ADT is working as intended.
**
***********************************************/

#include <iostream>
#include <sstream>
#include "graph.h"

using std::cout, std::endl;

int main() {
    { //Scope for example tests
    std::cout << "make an empty digraph" << std::endl;

    Graph G;

    G.add_vertex(1);
    G.add_vertex(2);
    G.add_edge(1,2);
    G.dijkstra(1);
    cout << "Cost: " << G.cost(1, 2) << endl;
    cout << "Distance: " << G.distance(1) << endl;
    cout << "Cost: " << G.cost(2, 1) << endl;
    cout << "Distance: " << G.distance(2) << endl;

    std::cout << "add vertices" << std::endl;
    for (size_t n = 1; n <= 7; n++) {
        G.add_vertex(n);
    }
    G.add_vertex(8);
    std::cout << "add directed edges" << std::endl;
    G.add_edge(1,2,5);  // 1 ->{5} 2; (edge from 1 to 2 with weight 5)
    G.add_edge(1,3,3);
    G.add_edge(2,3,2);
    G.add_edge(2,5,3);
    G.add_edge(2,7,1);
    G.add_edge(3,4,7);
    G.add_edge(3,5,7);
    G.add_edge(4,1,2);
    G.add_edge(4,6,6);
    G.add_edge(5,4,2);
    G.add_edge(5,6,1);
    G.add_edge(7,5,1);
    std::cout << "G has " << G.vertex_count() << " vertices" << std::endl;
    std::cout << "G has " << G.edge_count() << " edges" << std::endl;
    std::cout << std::endl;
    std::cout << "compute mst path from 2" <<std::endl;
    G.prim(2);
    std::cout << "print minimum spanning paths" <<std::endl;
    for (size_t n = 1; n <= 7; n++) {
        std::cout << "minimum spanning path from 8 to " << n << std::endl;
        std::cout << "  ";
        G.print_path(n);
    }
    std::cout << std::endl;
    std::cout << "compute shortest path from 2" <<std::endl;
    G.dijkstra(2);
    std::cout << "print shortest paths" <<std::endl;
    for (size_t n = 1; n <= 7; n++) {
        std::cout << "shortest path from 2 to " << n << std::endl;
        std::cout << "  ";
        G.print_shortest_path(n);
    }
    } //End example tests scope



    { //New scope for student tests
    Graph G;
    size_t N = 4;
    for(size_t id = 1; id <= N; id++){
        G.add_vertex(id);
    }
    size_t cnt = 0;
    for(size_t src = 1; src <= N; src++){
        for(size_t dest = 1; dest <= N; dest++){
            cout << "ITERATION: src = " << src << " dest = " << dest << endl;
            if(dest == src){
                continue;
            }
            if(G.edge_count() == cnt){
                cout << "PASS: edge count currently equals cnt" << endl;
            } else {
                cout << "FAIL: edge count equal " << G.edge_count() << " when it should equal " << cnt << endl;
            }
            cout << endl;
            if(!G.contains_edge(src, dest)){
                cout << "PASS: contains edge works for src = " << src << " and dest = " << dest << endl;
            } else {
                cout << "FAIL: contains edge doesn't work for src = " << src << " and dest = " << dest << endl;
            }
            cout << endl;
            if(G.add_edge(src, dest)){
                cout << "PASS: add edge works for src = " << src << " and dest = " << dest << endl;
            } else {
                cout << "FAIL: add edge doesn't work for src = " << src << " and dest = " << dest << endl;
            }
            cnt++;
            cout << "Incremented count " << endl;
            if(G.edge_count() == cnt){
                cout << "PASS: edge count works for edge count = " << G.edge_count() << " and cnt = " << cnt << endl;
            } else {
                cout << "FAIL: edge count doesn't work for edge count = " << G.edge_count() << " and cnt = " << cnt << endl;
            }
            cout << endl;
            if(G.contains_edge(src, dest)){
                cout << "PASS: contains edge works for src = " << src << " and dest = " << dest << endl;
            } else {
                cout << "FAIL: contains edge doesn't work for src = " << src << " and dest = " << dest << endl;
            }
            if(std::abs(G.cost(src, dest)) - 1 <= 1e-6) {
                cout << "PASS: cost works for src = " << src << " and dest = " << dest << endl;
            } else {
                cout << "FAIL: cost doesn't work for src = " << src << " and dest = " << dest << endl;
            }
            if(!G.add_edge(src, dest)){
                cout << "PASS: add edge works for src = " << src << " and dest = " << dest << endl;
            } else {
                cout << "FAIL: add edge doesn't work for src = " << src << " and dest = " << dest << endl;
            }
            cout << "After add edge" << endl;
            if(G.edge_count() == cnt){
                cout << "PASS: edge count works for edge count = " << G.edge_count() << " and cnt = " << cnt << endl;
            } else {
                cout << "FAIL: edge count doesn't work for edge count = " << G.edge_count() << " and cnt = " << cnt << endl;
            }            
        }
    }
    } //End student tests scope

    return 0;
}