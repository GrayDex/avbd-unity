using UnityEngine;

namespace Avbd
{
    // A box in the world, sized in its own axes. Density 0 makes it static: the solver reads its transform
    // every step instead of writing it, so it can be animated.
    [DisallowMultipleComponent]
    public class AvbdBody : MonoBehaviour
    {
        [SerializeField, Tooltip("Full widths, metres.")] private Vector3 size = Vector3.one;
        [SerializeField, Min(0f), Tooltip("kg/m³; 0 — static.")] private float density = 1f;
        [SerializeField, Min(0f)] private float friction = 0.5f;
        [SerializeField, Tooltip("Initial velocity, m/s.")] private Vector3 velocity;

        public int Id { get; private set; } = -1;
        public AvbdWorld World { get; private set; }

        public Vector3 Size
        {
            get => size;
            set => size = value;
        }

        public float Density
        {
            get => density;
            set => density = value;
        }

        public float Friction
        {
            get => friction;
            set => friction = value;
        }

        public Vector3 Velocity
        {
            get => velocity;
            set => velocity = value;
        }

        // Joins the world now; joints call it so they can link bodies whose OnEnable has not run yet.
        public bool Register()
        {
            if (Id >= 0) return true;
            World = AvbdWorld.Of(this);
            if (!World) return false;
            Id = World.Solver.AddBox(size, density, friction, transform.position, transform.rotation, velocity);
            World.Add(this);
            return true;
        }

        private void OnEnable() => Register();

        private void OnDisable()
        {
            if (Id < 0) return;
            if (World && World.Alive)
            {
                World.Solver.RemoveBody(Id);
                World.Remove(this);
            }

            Id = -1;
        }

        internal void Push()
        {
            ref var body = ref World.Solver.Get(Id);
            if (!body.Static) return;
            body.Position = transform.position;
            body.Rotation = transform.rotation;
        }

        internal void Pull()
        {
            var body = World.Solver.Get(Id);
            if (!body.Static) transform.SetPositionAndRotation(body.Position, body.Rotation);
        }
    }
}
