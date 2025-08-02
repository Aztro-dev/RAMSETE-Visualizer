#include <thread>

#define PATH "paths/awp.txt"

extern void pid_turn();

std::vector<int> reverse_indices = {3, 5, 8, 10};

void run(int current_node) {
  if (current_node == 4) {
    pid_turn();
  }
  if (current_node == 5) {
    printf("outtaking...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (current_node == 6) {
    pid_turn();
  }
  if (current_node == 7) {
    printf("intaking...\n");
    pid_turn();
  }
  if (current_node == 8) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (current_node == 9) {
    pid_turn();
  }
  if (current_node == 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (current_node == 11) {
    pid_turn();
  }
}
