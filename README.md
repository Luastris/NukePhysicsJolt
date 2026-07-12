# NukePhysicsJolt

Physics module for [NukeEngine](https://github.com/Luastris/NukeEngine-Eco), built on
[Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.5 (vendored in `deps/`, only the
library source tracked). Implements the engine's `iPhysics` POD seam — the engine core
contains zero Jolt types.

## Features

- `Collider` (box/sphere/capsule/mesh, trigger flag) + `Rigidbody` (mass, forces,
  velocities, angular) components — reflected, so they serialize and edit like anything
  else.
- Fixed-step simulation driven by the World on its own thread; the Jolt solver runs on
  the engine's Jobs pool (multithreaded stepping).
- Contact/trigger callbacks dispatched to script components
  (`onCollisionEnter/Exit`, `onTriggerEnter/Exit`).
- Queries through the reflected `nuke::Physics` facade — usable from C++, Lua
  (`nuke.Physics.*`) and C# (`Physics.*`): raycasts, shape casts, overlaps.
- Externally moved transforms (gizmo, scripts) re-drive their bodies; editor debug
  drawing shows volumes and triggers.

```lua
if nuke.Physics.Raycast(from, dir, 100) then
    local atom = nuke.Physics.HitAtom()
    local dist = nuke.Physics.HitDistance()
end
```

## Building

Part of the [NukeEngine-Eco](https://github.com/Luastris/NukeEngine-Eco) superbuild, or
standalone: `cmake -S . -B build -G "Visual Studio 17 2022" -A x64` +
`cmake --build build --config Debug` (needs `VCPKG_ROOT`; engine first).
