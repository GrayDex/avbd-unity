using System.Collections.Generic;
using UnityEngine;

namespace Avbd
{
    // Owns a Solver, steps it every FixedUpdate and moves the transforms of its bodies.
    // Bodies, joints and springs below it (or anywhere, if it is the only world) join it by themselves.
    [DefaultExecutionOrder(-100)]
    public class AvbdWorld : MonoBehaviour
    {
        [SerializeField] private Vector3 gravity = new(0f, -10f, 0f);
        [SerializeField, Range(1, 50)] private int iterations = 10;
        [SerializeField, Range(0f, 1f), Tooltip("Share of the step-start error kept by hard rows: 1 leaves it, 0 removes it at once.")]
        private float alpha = 0.99f;
        [SerializeField, Tooltip("Penalty ramp of linear rows.")] private float betaLin = 10000f;
        [SerializeField, Tooltip("Penalty ramp of angular rows.")] private float betaAng = 100f;
        [SerializeField, Range(0f, 1f), Tooltip("Warm-start decay of forces and penalties between steps.")] private float gamma = 0.999f;

        private Solver _solver;
        private readonly List<AvbdBody> _bodies = new();

        public Solver Solver => _solver ??= new Solver();
        public bool Alive => _solver != null;

        public Vector3 Gravity
        {
            get => gravity;
            set => gravity = value;
        }

        public int Iterations
        {
            get => iterations;
            set => iterations = value;
        }

        public static AvbdWorld Of(Component component)
        {
            var near = component.GetComponentInParent<AvbdWorld>();
            return near ? near : FindAnyObjectByType<AvbdWorld>();
        }

        internal void Add(AvbdBody body) => _bodies.Add(body);

        internal void Remove(AvbdBody body) => _bodies.Remove(body);

        private void FixedUpdate()
        {
            var solver = Solver;
            solver.Settings = new Settings
            {
                Dt = Time.fixedDeltaTime, Gravity = gravity, Iterations = iterations,
                Alpha = alpha, BetaLin = betaLin, BetaAng = betaAng, Gamma = gamma
            };
            foreach (var body in _bodies) body.Push();
            solver.Step();
            foreach (var body in _bodies) body.Pull();
        }

        private void OnDestroy()
        {
            _solver?.Dispose();
            _solver = null;
        }
    }
}
