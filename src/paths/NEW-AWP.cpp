#include <iostream>
#include <thread>
#include <vector>
#define PATH "paths/NEW-AWP.txt"

extern void pid_turn();

std::vector<int> reverse_indices = {4, 5, 8};

void run(int current_node) {
  if (current_node == 1) {
    printf("intaking...\n");
  }
  if (current_node == 2) {
    printf("still intaking...\n");
  }
  if (current_node == 3) {
    printf("intaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_node == 5) {
    pid_turn();
  }
  if (current_node == 7) {
    pid_turn();
  }
  if (current_node == 8) {
    printf("outtaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_node == 10) {
    pid_turn();
  }
  if (current_node == 11) {
    printf("intaking...\n");
  }
  if (current_node == 12) {
    pid_turn();
  }
  if (current_node == 13) {
    printf("outtaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
