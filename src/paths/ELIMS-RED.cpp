#include <iostream>
#include <thread>
#include <vector>
#define PATH "paths/ELIMS_RED.txt"

extern void pid_turn();

std::vector<int> reverse_indices = {3, 5, 9};

void run(int current_node) {
  if (current_node == 2) {
    printf("intaking...\n");
  }
  if (current_node == 3) {
    printf("intaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  if (current_node == 4) {
    pid_turn();
  }
  if (current_node == 5) {
    printf("outtaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_node == 6) {
    pid_turn();
  }
  if (current_node == 7) {
    printf("intaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  if (current_node == 8) {
    pid_turn();
  }
  if (current_node == 9) {
    printf("outtaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (current_node == 10) {
    pid_turn();
  }
  if (current_node == 11) {
    pid_turn();
  }

  if (current_node == 12) {
    pid_turn();
  }
  if (current_node == 13) {
    printf("intaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}
