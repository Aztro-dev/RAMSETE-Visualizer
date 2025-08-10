# RAMSETE Path Following Visualizer

A real-time tool for testing and tuning RAMSETE path-following algorithms for differential drive robots. Simulates robot motion with accurate kinematics and provides a visual interface for path tracking and tuning.

## Features
- Real-time RAMSETE controller simulation  
- Live path tracking (planned vs actual)  
- Optional motor acceleration & RPM limits  
- Multi-threaded control & rendering  
- CSV path loading for Path Planner at: (https://path.jerryio.com)  
- Interactive pause/tuning controls  
- Node-based waypoints (rotate/reverse support)  

## Project Structure
```
src/        # Core simulation and control code
paths/      # Path behaviors & CSVs
resources/  # Robot sprite & field background
Makefile
```

## Dependencies
- Raylib — see [Raylib GitHub Releases](https://github.com/raysan5/raylib/releases)  
- C++17  

**Install Raylib**  
```bash
# Ubuntu/Debian
sudo apt install libraylib-dev
# macOS
brew install raylib
# Windows (vcpkg)
vcpkg install raylib

- C++17  


## Build & Run
```bash
make      
make run
make clean 
```

## Usage
- **Run** — `make run`  
- **Pause/Resume** — Press SPACE  
- **Nodes** — Blue circles with numbers  

**Configure paths** in `src/control.hpp`:
```cpp
#include "paths/your_path.cpp"
std::vector<int> rotating_indices = {4, 9, 13}; // when you want to rotate
std::vector<int> reverse_indices  = {6, 7}; // when you want to reverse
```

**Tune parameters** in `constants.cpp`:
```cpp
#define B 2.0
#define ZETA 0.7
#define TRACK_WIDTH_M 0.381
#define WHEEL_RADIUS_M 0.0508
#define MAX_SPEED_OUTPUT 500.0
```


## Real Robot Deployment
Use:
- `ramsete.hpp`
- `path.hpp`
- `motor.hpp`
- `constants.hpp`
- `control.hpp`  

Remove:
- `main.cpp`  
- `color.cpp`  
- Raylib dependencies
