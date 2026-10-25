# AVBD Physics for Unity

[Augmented Vertex Block Descent](https://graphics.cs.utah.edu/research/projects/avbd/) (Giles, Diaz, Yuksel, SIGGRAPH 2025) —
rigid bodies, joints, springs and frictional contacts in one primal–dual solver, Burst-compiled.

It is a port of the authors' [avbd-demo3d](https://github.com/savant117/avbd-demo3d), and it matches it **bit for bit**:
`Tests/Editor/ReferenceTests` runs the demo's 13 scenes for 600 steps and compares every body's pose with poses dumped
by the C++ demo itself (`Tools~/Reference`).

## Install

Package Manager → **+** → *Add package from git URL*:

```
https://github.com/GrayDex/avbd-unity.git
```

Requires Unity 6, Burst, Collections, Mathematics. Samples → *Demo scenes* imports the demo's scenes with its panel.

## Use

Components: put an `AvbdWorld` on a root object and `AvbdBody` (a box or a point mass; density, or mass for a point,
0 is static and follows its transform) on its children. `AvbdJoint` links a body to another or to a world point; `AvbdSpring` is a linear spring.
The world steps in `FixedUpdate` and writes the transforms back.

Or the solver directly:

```csharp
using var solver = new Avbd.Solver();
var ground = solver.AddBox(new float3(100, 1, 100), 0f, 0.5f, float3.zero, quaternion.identity);
var box = solver.AddBox(1f, 1f, 0.5f, new float3(0, 4, 0), quaternion.identity);
solver.AddJoint(-1, box, new float3(0, 8, 0), new float3(0, 0.5f, 0)); // hang it from a world point
solver.Step();
var pose = solver.Get(box).Position;
```

Joint stiffness: infinite is a hard row, zero a free one — linear ∞ and angular 0 is a ball joint, both ∞ a weld,
finite values make springs. A fracture torque breaks the joint.

Angle limits (`Solver.LimitJoint`, or the *Angle limits* fields of `AvbdJoint`, laid out as in `ConfigurableJoint`):
the joint frame's relative rotation splits into twist about x, then swing about y and z, as in PhysX D6. Each axis is
free, limited to [lower, upper] or locked; a bound is a hard row whose force is clamped to one side (paper §3.2,
stiffness rescaling of Eq. 14). Twist limits alone leave a ball joint free to swing round past them — lock or limit
the swings for a hinge. `Tests/Editor/LimitTests` checks a pendulum against them.

## Status

Point masses (`Solver.AddPoint`) have 3 degrees of freedom: joints and springs hold them at their position (anchor
zero), angular rows and limits on them are refused, and they collide with nothing.

Single-threaded like the demo: the whole step is one Burst job. Collisions between boxes only. Planned: capsules against
triangles of weighted point masses, parallel solve by graph colouring.

## License

MIT. Ported from avbd-demo3d © 2025 Chris Giles (MIT).
