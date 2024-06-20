CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Weffc++ -pedantic-errors -g

objects = graph

all:  $(objects)

memory_errors: graph_memory_errors

compile_test: graph_compile_test

$(objects): %: clean %.h %_tests.cpp
	g++ $(CXXFLAGS) --coverage $@_tests.cpp && ./a.out && rm -f *.gcov *.gcda *.gcno a.out

graph_memory_errors: %_memory_errors: clean %.h %_tests.cpp
	g++ $(CXXFLAGS) graph_tests.cpp && valgrind --leak-check=full ./a.out
	g++ $(CXXFLAGS) $@.cpp && valgrind --leak-check=full ./a.out

graph_compile_test: %_compile_test: %.h %_compile_test.cpp
	g++ $(CXXFLAGS) $@.cpp

clean:
	rm -f *.gcov *.gcda *.gcno a.out
