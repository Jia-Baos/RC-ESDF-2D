# AGENTS.md - Agent Coding Guidelines for RC-ESDF-2D

## Project Overview

RC-ESDF-2D is a C++ implementation of the Robo-Centric Euclidean Signed Distance Field algorithm for robot path planning. It provides fast O(1) distance and gradient queries for collision avoidance.

- **Language**: C++14/17
- **Build System**: CMake (>= 3.10)
- **Dependencies**: Eigen3, OpenCV

---

## Build Commands

### Full Build
```bash
mkdir -p build && cd build
cmake ..
make
```

### Running the Application
```bash
./build/test_rc_esdf
```

### Single Test Execution
There is no formal test framework. The `main.cpp` file serves as the test/demo executable. To run a specific test:
1. Modify the test code in `main.cpp` to isolate the specific functionality
2. Rebuild with `make` in the build directory
3. Run `./build/test_rc_esdf`

### Clean Build
```bash
rm -rf build && mkdir build && cd build && cmake .. && make
```

---

## Code Style Guidelines

### File Organization
- Header files: `include/` directory with `.h` extension
- Source files: `src/` directory with `.cpp` extension
- Use include guards: `#ifndef RC_ESDF_H` / `#define RC_ESDF_H` / `#endif`

### Naming Conventions
- **Classes**: CamelCase (e.g., `RcEsdfMap`)
- **Functions**: camelCase (e.g., `generateFromPolygon`, `query`)
- **Member Variables**: trailing underscore (e.g., `width_m_`, `resolution_`, `data_`)
- **Constants**: camelCase with descriptive prefixes (e.g., `max_inner`, `grid_size_x_`)
- **Files**: lowercase with underscores (e.g., `rc_esdf.h`, `rc_esdf.cpp`)

### Code Formatting
- **Indentation**: 4 spaces (no tabs)
- **Braces**: Opening brace on same line, closing brace on own line
- **Line Length**: Keep lines reasonable (< 100 characters when possible)
- **Spaces**: Use spaces around operators and after commas

### Header Organization
Order includes as:
1. Matching header (for .cpp files)
2. Standard library headers (`<vector>`, `<cmath>`, etc.)
3. Third-party headers (`<Eigen/Core>`, OpenCV)
4. Project headers (`"rc_esdf.h"`)

Example (.cpp file):
```cpp
#include "rc_esdf.h"
#include <limits>
#include <opencv2/opencv.hpp>
```

### Types and Variables
- Use `double` for floating-point calculations
- Use `int` for grid indices and sizes
- Use `float` for ESDF data storage (`std::vector<float>`)
- Use Eigen types (`Eigen::Vector2d`) for 2D points and vectors
- Prefer `const` for parameters that should not be modified

### Comments
- Use Doxygen-style comments for public API:
```cpp
/**
 * @brief Brief description
 * @param param_name [input] description
 * @return description
 */
```
- Use Chinese or English consistently (current codebase uses Chinese comments)
- Comment complex logic but avoid obvious comments

### Error Handling
- Return `bool` from query functions to indicate success/failure
- Use `false` for out-of-bounds queries
- Set output parameters to safe defaults (e.g., `grad.setZero()`, `dist = 0.0`) on failure

### Performance Considerations
- Use inline functions for simple helpers (e.g., `posToGrid`, `getRaw`)
- Use `std::vector` with `reserve()` when size is known
- Prefer squared distances (`squaredNorm()`) when comparing distances
- Minimize IO operations in performance-critical loops

---

## Project Structure

```
RC-ESDF-2D/
├── include/
│   └── rc_esdf.h      # Public API and class definition
├── src/
│   └── rc_esdf.cpp    # Implementation
├── main.cpp           # Test/demo executable
├── CMakeLists.txt     # Build configuration
├── README.md          # Documentation
└── LICENSE            # MIT License
```

---

## Common Tasks

### Adding a New Method to RcEsdfMap
1. Declare in `include/rc_esdf.h` in the public section
2. Implement in `src/rc_esdf.cpp`
3. Add test case in `main.cpp`
4. Rebuild with `make`

### Modifying the ESDF Generation Algorithm
- Edit `RcEsdfMap::generateFromPolygon()` in `src/rc_esdf.cpp`
- This runs once at startup (offline phase)

### Modifying the Query Algorithm
- Edit `RcEsdfMap::query()` in `src/rc_esdf.cpp`
- This runs online for each obstacle point

### Adding Visualization
- Use OpenCV functions (see `visualizeEsdf()` for examples)
- Common: `cv::imshow()`, `cv::waitKey()`, `cv::line()`, `cv::rectangle()`

---

## Notes for Agents

- This is a small, single-purpose library with no external test framework
- The `main.cpp` file is both a demo and test harness
- There are no linting or formatting tools configured
- Follow the existing code style when making modifications
- The codebase uses Chinese comments - maintain consistency
- Always rebuild after modifying code: `cd build && make`
