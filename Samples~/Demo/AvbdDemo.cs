using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Avbd.Samples
{
    // The scenes of avbd-demo3d built from AvbdBody / AvbdJoint / AvbdSpring, with the demo's panel:
    // scene picker, gravity, step, iterations. Left drag pulls a box on a joint, right drag orbits, wheel zooms.
    // Scenes are written in the demo's numbers (z up) and turned to Unity's y up when built.
    [RequireComponent(typeof(AvbdWorld))]
    public class AvbdDemo : MonoBehaviour
    {
        [SerializeField] private int scene = 4;

        private const float DragStiffness = 5000f;

        private AvbdWorld _world;
        private Transform _root;
        private Camera _camera;
        private Material _dynamic, _static;
        private Rect _panel = new(10, 10, 230, 0);
        private float _yaw = 30f, _pitch = 20f, _distance = 28f;
        private int _drag = -1, _dragBody = -1;
        private float _dragDistance;
        private float _savedStep;
        private Vector2 _mouse; // screen pixels, from GUI events: works with either input backend

        private static readonly (string Name, Action<AvbdDemo> Build)[] Scenes =
        {
            ("Empty", _ => { }), ("Ground", d => d.Ground()), ("Dynamic Friction", d => d.DynamicFriction()),
            ("Static Friction", d => d.StaticFriction()), ("Pyramid", d => d.Pyramid()), ("Rope", d => d.Rope()),
            ("Heavy Rope", d => d.HeavyRope()), ("Spring", d => d.Spring()), ("Spring Ratio", d => d.SpringsRatio()),
            ("Stack", d => d.Stack()), ("Stack Ratio", d => d.StackRatio()), ("Soft Body", d => d.SoftBody()),
            ("Bridge", d => d.Bridge()), ("Breakable", d => d.Breakable())
        };

        private void Awake()
        {
            _world = GetComponent<AvbdWorld>();
            _savedStep = Time.fixedDeltaTime;
            Time.fixedDeltaTime = 1f / 60f; // the demo's step
            _camera = Camera.main;
            if (!_camera) _camera = new GameObject("Camera", typeof(Camera)) { tag = "MainCamera" }.GetComponent<Camera>();
            if (!FindAnyObjectByType<Light>())
                new GameObject("Light", typeof(Light)) { transform = { rotation = Quaternion.Euler(50f, -30f, 0f) } }.GetComponent<Light>().type = LightType.Directional;

            var shader = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _dynamic = new Material(shader.GetComponent<Renderer>().sharedMaterial) { color = new Color(0.85f, 0.55f, 0.3f) };
            _static = new Material(_dynamic) { color = new Color(0.45f, 0.47f, 0.5f) };
            Destroy(shader);
        }

        private void Start() => Load(scene);

        private void OnDestroy() => Time.fixedDeltaTime = _savedStep;

        private void Load(int index)
        {
            scene = index;
            EndDrag();
            if (_root) DestroyImmediate(_root.gameObject);
            _root = new GameObject(Scenes[index].Name).transform;
            _root.SetParent(transform, false);
            Scenes[index].Build(this);
        }

        private void LateUpdate()
        {
            var orbit = Quaternion.Euler(_pitch, _yaw, 0f);
            var target = new Vector3(0f, 5f, 0f);
            _camera.transform.SetPositionAndRotation(target - orbit * Vector3.forward * _distance, orbit);
        }

        private void FixedUpdate()
        {
            if (_drag < 0) return;
            if (!_world.Solver.ContainsForce(_drag) || !_world.Solver.Contains(_dragBody))
            {
                EndDrag();
                return;
            }

            var ray = _camera.ScreenPointToRay(_mouse);
            _world.Solver.GetForce(_drag).AnchorA = ray.origin + ray.direction * _dragDistance;
        }

        private void OnGUI()
        {
            _panel = GUILayout.Window(0, _panel, Panel, "AVBD");
            var e = Event.current;
            var inside = _panel.Contains(e.mousePosition);
            if (e.isMouse) _mouse = new Vector2(e.mousePosition.x, Screen.height - e.mousePosition.y);
            switch (e.type)
            {
                case EventType.MouseDown when e.button == 0 && !inside: BeginDrag(); break;
                case EventType.MouseUp when e.button == 0: EndDrag(); break;
                case EventType.MouseDrag when e.button == 1:
                    _yaw += e.delta.x * 0.3f;
                    _pitch = Mathf.Clamp(_pitch + e.delta.y * 0.3f, -85f, 85f);
                    break;
                case EventType.ScrollWheel when !inside: _distance = Mathf.Clamp(_distance * (1f + e.delta.y * 0.05f), 2f, 200f); break;
            }
        }

        private void Panel(int id)
        {
            for (var i = 0; i < Scenes.Length; i++)
                if (GUILayout.Toggle(i == scene, Scenes[i].Name, "Button") && i != scene)
                    Load(i);
            if (GUILayout.Button("Reset")) Load(scene);

            GUILayout.Label($"Gravity {_world.Gravity.y:F1}");
            _world.Gravity = new Vector3(0f, GUILayout.HorizontalSlider(_world.Gravity.y, -20f, 20f), 0f);
            GUILayout.Label($"Dt {Time.fixedDeltaTime:F4}");
            Time.fixedDeltaTime = GUILayout.HorizontalSlider(Time.fixedDeltaTime, 0.001f, 0.1f);
            GUILayout.Label($"Iterations {_world.Iterations}");
            _world.Iterations = Mathf.RoundToInt(GUILayout.HorizontalSlider(_world.Iterations, 1f, 50f));
            var solver = _world.Solver;
            GUILayout.Label($"Bodies {solver.BodyCount}, forces {solver.ForceCount}, contacts {solver.ContactCount}");
            GUI.DragWindow();
        }

        private void BeginDrag()
        {
            var ray = _camera.ScreenPointToRay(_mouse);
            var solver = _world.Solver;
            var body = solver.Pick(ray.origin, ray.direction, out var local);
            if (body < 0) return;
            var hit = math.rotate(solver.Get(body).Rotation, local) + solver.Get(body).Position;
            _dragDistance = math.max(math.dot(hit - (float3)ray.origin, ray.direction), 0.1f);
            _dragBody = body;
            _drag = solver.AddJoint(-1, body, hit, local, DragStiffness);
        }

        private void EndDrag()
        {
            if (_drag >= 0 && _world.Alive) _world.Solver.RemoveForce(_drag);
            _drag = _dragBody = -1;
        }

        // The demo's frame is z up; Unity's is y up: (x, y, z) → (x, z, −y), a rotation, so handedness is kept.
        private static Vector3 Point(float3 p) => new(p.x, p.z, -p.y);

        private static Vector3 Extent(float3 s) => new(s.x, s.z, s.y);

        private AvbdBody Box(float3 size, float density, float friction, float3 position, float3 velocity = default, quaternion rotation = default)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(_root, false);
            go.transform.SetPositionAndRotation(Point(position), rotation.Equals(default) ? Quaternion.identity : rotation);
            go.transform.localScale = Extent(size);
            go.GetComponent<Renderer>().sharedMaterial = density > 0f ? _dynamic : _static;
            go.SetActive(false);
            var body = go.AddComponent<AvbdBody>();
            body.Size = Extent(size);
            body.Density = density;
            body.Friction = friction;
            body.Velocity = Point(velocity);
            go.SetActive(true);
            return body;
        }

        private void Floor(float z = 0f) => Box(new float3(100, 100, 1), 0f, 0.5f, new float3(0, 0, z));

        private static void Joint(AvbdBody a, AvbdBody b, float3 rA, float3 rB, float linear = float.PositiveInfinity, float angular = 0f,
            float fracture = float.PositiveInfinity) =>
            b.gameObject.AddComponent<AvbdJoint>().Connect(a, Point(rB), Point(rA), linear, angular, fracture);

        private static void Spring(AvbdBody a, AvbdBody b, float3 rA, float3 rB, float stiffness, float rest) =>
            b.gameObject.AddComponent<AvbdSpring>().Connect(a, Point(rB), Point(rA), stiffness, rest);

        private void Ignore(AvbdBody a, AvbdBody b) => _world.Solver.IgnoreCollision(a.Id, b.Id);

        private void Ground()
        {
            Floor();
            Box(1, 1f, 0.5f, new float3(0, 0, 4));
        }

        private void DynamicFriction()
        {
            Floor();
            for (var x = 0; x <= 10; x++)
                Box(new float3(1, 1, 0.5f), 1f, 5f - x / 10f * 5f, new float3(0, -30f + x * 2f, 0.75f), new float3(10f, 0, 0));
        }

        private void StaticFriction()
        {
            Floor();
            const float angle = 30f * Mathf.Deg2Rad;
            var demoRotation = quaternion.AxisAngle(new float3(0, 1, 0), angle);
            var ramp = Box(new float3(40, 24, 1), 0f, 1f, new float3(0, 0, 3), default, quaternion.AxisAngle(new float3(0, 0, -1), angle));
            var tangent = math.rotate(demoRotation, new float3(1, 0, 0));
            var normal = math.rotate(demoRotation, new float3(0, 0, 1));
            var center = new float3(ramp.transform.position.x, -ramp.transform.position.z, ramp.transform.position.y);
            for (var i = 0; i <= 10; i++)
                Box(1, 1f, i / 10f * 0.25f + 0.25f, center + tangent * -12f + new float3(0, -10f + i * 2f, 0) + normal * 1.05f);
        }

        private void Pyramid()
        {
            const int size = 16;
            Floor(-0.5f);
            for (var y = 0; y < size; y++)
            for (var x = 0; x < size - y; x++)
                Box(new float3(1, 0.5f, 0.5f), 1f, 0.5f, new float3(x * 1.01f + y * 0.5f - size / 2f, 0f, y * 0.85f + 0.5f));
        }

        private void Rope()
        {
            Floor(-20f);
            AvbdBody previous = null;
            for (var i = 0; i < 20; i++)
            {
                var current = Box(new float3(1, 0.5f, 0.5f), i == 0 ? 0f : 1f, 0.5f, new float3(i, 0f, 10f));
                if (previous) Joint(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0));
                previous = current;
            }
        }

        private void HeavyRope()
        {
            const int n = 20;
            const float size = 5f;
            Floor(-20f);
            AvbdBody previous = null;
            for (var i = 0; i < n; i++)
            {
                var last = i == n - 1;
                var current = Box(last ? new float3(size) : new float3(1, 0.5f, 0.5f), i == 0 ? 0f : 1f, 0.5f, new float3(i + (last ? size / 2 : 0), 0f, 10f));
                if (previous) Joint(previous, current, new float3(0.5f, 0, 0), last ? new float3(-size / 2, 0, 0) : new float3(-0.5f, 0, 0));
                previous = current;
            }
        }

        private void Spring()
        {
            Floor();
            var anchor = Box(1, 0f, 0.5f, new float3(0, 0, 14f));
            var block = Box(2, 1f, 0.5f, new float3(0, 0, 8f));
            Spring(anchor, block, float3.zero, float3.zero, 100f, 4f);
        }

        private void SpringsRatio()
        {
            const int n = 8;
            Floor(-10f);
            AvbdBody previous = null;
            for (var i = 0; i < n; i++)
            {
                var current = Box(new float3(1, 0.75f, 0.75f), i is 0 or n - 1 ? 0f : 1f, 0.5f, new float3((i - (n - 1) * 0.5f) * 3f, 0f, 12f));
                if (previous) Spring(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), i % 2 == 0 ? 10f : 10000f, 3f);
                previous = current;
            }
        }

        private void Stack()
        {
            Floor();
            for (var i = 0; i < 10; i++) Box(1, 1f, 0.5f, new float3(0, 0, i * 1.5f + 1f));
        }

        private void StackRatio()
        {
            Floor();
            var top = 0.5f;
            var size = 1f;
            for (var i = 0; i < 4; i++)
            {
                Box(size, 1f, 0.5f, new float3(0, 0, top + size * 0.5f));
                top += size;
                size *= 2f;
            }
        }

        private void SoftBody()
        {
            const float lin = 1000f, ang = 250f, size = 0.8f, half = size * 0.5f;
            const int w = 4, d = 4, h = 4;
            Floor();
            for (var i = 0; i < 3; i++)
            {
                var grid = new AvbdBody[w, d, h];
                var stackZ = i * (h * size + 2f);
                for (var x = 0; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 0; z < h; z++)
                    grid[x, y, z] = Box(size, 1f, 0.5f, new float3((x - (w - 1) * 0.5f) * size, (y - (d - 1) * 0.5f) * size, 8f + stackZ + z * size));

                var links = new List<(AvbdBody, AvbdBody)>();
                for (var x = 0; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 0; z < h; z++)
                {
                    if (x > 0) Joint(grid[x - 1, y, z], grid[x, y, z], new float3(half, 0, 0), new float3(-half, 0, 0), lin, ang);
                    if (y > 0) Joint(grid[x, y - 1, z], grid[x, y, z], new float3(0, half, 0), new float3(0, -half, 0), lin, ang);
                    if (z > 0) Joint(grid[x, y, z - 1], grid[x, y, z], new float3(0, 0, half), new float3(0, 0, -half), lin, ang);
                    if (x > 0 && z > 0) links.Add((grid[x - 1, y, z - 1], grid[x, y, z]));
                    if (x > 0 && z > 0) links.Add((grid[x, y, z - 1], grid[x - 1, y, z]));
                    if (y > 0 && z > 0) links.Add((grid[x, y - 1, z - 1], grid[x, y, z]));
                    if (y > 0 && z > 0) links.Add((grid[x, y, z - 1], grid[x, y - 1, z]));
                    if (x > 0 && y > 0) links.Add((grid[x - 1, y - 1, z], grid[x, y, z]));
                    if (x > 0 && y > 0) links.Add((grid[x, y - 1, z], grid[x - 1, y, z]));
                }

                foreach (var (a, b) in links) Ignore(a, b);
            }
        }

        private void Bridge()
        {
            const int n = 40;
            const float halfLength = 0.5f, halfWidth = 2f;
            Floor();
            AvbdBody previous = null;
            for (var i = 0; i < n; i++)
            {
                var current = Box(new float3(1f, 4f, 0.5f), i is 0 or n - 1 ? 0f : 1f, 0.5f, new float3(i - n / 2f, 0f, 10f));
                if (previous)
                {
                    Joint(previous, current, new float3(halfLength, halfWidth, 0), new float3(-halfLength, halfWidth, 0));
                    Joint(previous, current, new float3(halfLength, -halfWidth, 0), new float3(-halfLength, -halfWidth, 0));
                }

                previous = current;
            }

            for (var x = 0; x < n / 4; x++)
            for (var y = 0; y < n / 8; y++)
                Box(1, 1f, 0.5f, new float3(x - n / 8f, 0f, y + 12f));
        }

        private void Breakable()
        {
            const int n = 10;
            Floor();
            AvbdBody previous = null;
            for (var i = 0; i <= n; i++)
            {
                var current = Box(new float3(1, 1, 0.5f), 1f, 0.5f, new float3(i - n / 2f, 0f, 6f));
                if (previous) Joint(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), float.PositiveInfinity, float.PositiveInfinity, 90f);
                previous = current;
            }

            Box(new float3(1, 1, 5), 0f, 0.5f, new float3(-n / 2f, 0, 2.5f));
            Box(new float3(1, 1, 5), 0f, 0.5f, new float3(n / 2f, 0, 2.5f));
            for (var i = 0; i < 5; i++) Box(new float3(2, 1, 1), 1f, 0.5f, new float3(0, 0, i * 2f + 8f));
        }
    }
}
