## Overview

This project involves implementing key graph algorithms on directed graphs (digraphs), specifically Prim's and Dijkstra's algorithms. This repository includes a Makefile and various compilation and testing utilities to ensure robustness and correctness of the implementation.

## Code

This repository includes:
* graph.h: Header file with the graph class definition.
* graph_compile_test.cpp: Compile tests for the graph implementation.
* graph_tests.cpp: Unit tests to ensure functional correctness.
* makefile: Facilitates building, testing, and cleaning project files.

## Compilation Instructions

Use the following make commands:

* make: Builds and tests all data structures.
* make <type>: Builds and tests a specific component, e.g., make graph.
* make clean: Cleans the directory by removing coverage files and executables.
* make memory_errors: Checks for memory errors using Valgrind.
* make compile_test: Runs a compile test.

## Project Features
* Directed Graph Implementation. Implements a data structure to store a directed graph with functionalities to add/remove vertices and edges, and checks graph properties like vertex existence and edge weight.
* Prim’s Algorithm. Extends the Graph class to include Prim’s algorithm to compute a minimum spanning tree starting from a specified vertex.
* Dijkstra’s Algorithm. Further extends the Graph class to implement Dijkstra’s algorithm for computing the shortest paths from a specified source vertex to all other vertices.

## Testing
Each task comes with a comprehensive test suite that covers at least 90% of the code. Tests verify both the invocation of functions and the correctness of their outcomes.

