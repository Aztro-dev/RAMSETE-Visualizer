#include <iostream>
#include <thread>
#include <vector>
#define PATH "paths/MCL-Test.txt"

extern void pid_turn();

std::vector<int> reverse_indices = {};

void run(int current_node) {
  if (current_node == 3) {
    pid_turn();
  }
}
