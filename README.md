# Gravity Simulator

a real-time gravity simulator built with OpenGL

## Features
- real-time physics: gravitational forces between bodies using verlet integration
- visualization: smooth rendering with camera rotation
- controls: pause, speed up/down time, zoom in/out, and reset
- body types: central star, with inner planets, outer planets and small debris

## Controls
| Key   | Action             |
| Space | pause/resume       |
|↑ / ↓  | speed up/down time |
|← / →  | zoom in/out        |
|R      | reset              |
|Esc    | quit               |

### Build Requirements

- OpenGL 3.3+
- GLFW3 - window management
- GLEW - opengl extensions
- GLM - math library for graphics
- c++11 compatible compiler
- CMake
- Ninja

## Arch

```bash
sudo pacman -S cmake ninja glfw glm glew
```

## Building

```bash
mkdir build && cd build
cmake -G Ninja ..
ninja
./bin/gravity-sim
```

## Customization

changes can be made in `setupScene()` in `simulation.cpp` to adjust number of bodies, sizes, position, and velocity


