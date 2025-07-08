# Gravity Simulator

a real-time gravity simulator built with OpenGL

## Features
- Real-time physics: gravitational forces between bodies using Verlet integration
- Modern OpenGL 3.3+ rendering: smooth, anti-aliased discs for celestial bodies
- Color and size coding: sun (orange, large), planets (blue, medium), asteroids (white, small)
- Animated starfield background with subtle twinkle
- Smooth, cinematic camera transitions and zoom
- Real-time HUD: legend, controls, and body info on hover
- Hover highlight: body glows and info appears in HUD
- Camera focus: click a body to smoothly center/follow it
- Collision detection: bodies merge or bounce when close
- Real-time mass/radius controls: adjust selected body with keyboard
- Clean, minimal UI and codebase

## Demo
![Gravity Simulator](./media/sample.gif)

## Controls

| Key         | Action                                 |
|-------------|----------------------------------------|
| `Space`     | Pause/resume simulation                |
| `W` / `S`   | Speed up / slow down time              |
| `A` / `D`   | Zoom in / out                          |
| `T`         | Toggle trajectory lines                |
| `B`         | Use Barnes-Hut algorithm               |
| `M`         | Use Fast Multipole Method (FMM)        |
| `N`         | Use Direct N-body algorithm            |
| `I`         | Print physical properties of objects   |
| `G`         | Toggle CPU/GPU (N-body only)           |
| `R`         | Reset simulation                       |
| `Esc`       | Exit                                   |
| `+` / `-`   | Increase/decrease mass (hover/focus)   |
| `[` / `]`   | Decrease/increase radius (hover/focus) |
| Mouse hover | Show info and highlight body           |
| Mouse click | Focus camera on body                   |

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

You can edit `setupScene()` in `simulation.cpp` to adjust the number of bodies, their sizes, positions, and velocities.

## Optimizations
- Supports Barnes-Hut and Fast Multipole Method (FMM) for faster force calculation.
- GPU acceleration for N-body algorithm (OpenGL compute shader).
