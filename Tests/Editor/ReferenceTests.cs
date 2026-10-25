using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace Avbd.Tests
{
    // The port against avbd-demo3d itself: Reference.bytes holds every body's pose in the demo's scenes,
    // dumped by Tools~/Reference (frames 1–10, then every 10th up to 600). The port must match it bit for bit:
    // any difference is a logic change, since chaotic scenes (pyramid, soft body) amplify one ulp within frames.
    public class ReferenceTests
    {
        private const string Path = "Packages/io.github.graydex.avbd/Tests/Editor/Reference.bytes";

        private struct Frame
        {
            public int Index;
            public float3[] Positions;
            public float4[] Rotations;
        }

        private static Dictionary<int, Frame[]> _reference;

        private static IEnumerable<int> Scenes()
        {
            for (var i = 0; i < DemoScenes.All.Length; i++) yield return i;
        }

        [TestCaseSource(nameof(Scenes))]
        public void MatchesDemo(int scene)
        {
            var frames = Load()[scene + 1];
            using var solver = DemoScenes.Create(scene);
            Assert.AreEqual(frames[0].Positions.Length, solver.BodyCount, "bodies");

            float position = 0f, rotation = 0f;
            var diverged = -1;
            var next = 0;
            for (var step = 1; next < frames.Length; step++)
            {
                solver.Step();
                if (frames[next].Index != step) continue;
                var frame = frames[next++];
                for (var b = 0; b < solver.BodyCount; b++)
                {
                    var body = solver.Bodies[b];
                    var error = math.length(body.Position - frame.Positions[b]);
                    var turn = math.min(math.length(body.Rotation.value - frame.Rotations[b]), math.length(body.Rotation.value + frame.Rotations[b]));
                    position = math.max(position, error);
                    rotation = math.max(rotation, turn);
                    if (diverged < 0 && (error > 0f || turn > 0f)) diverged = step;
                }
            }

            Debug.Log($"[Avbd] {DemoScenes.All[scene].Name}: position {position:E2} m, rotation {rotation:E2}, first difference at frame {diverged}");
            Assert.AreEqual(-1, diverged, "first frame that differs");
        }

        private static Dictionary<int, Frame[]> Load()
        {
            if (_reference != null) return _reference;
            using var reader = new BinaryReader(File.OpenRead(System.IO.Path.GetFullPath(Path)));
            _reference = new Dictionary<int, Frame[]>();
            var scenes = reader.ReadInt32();
            for (var s = 0; s < scenes; s++)
            {
                var id = reader.ReadInt32();
                var bodies = reader.ReadInt32();
                var frames = new Frame[reader.ReadInt32()];
                for (var f = 0; f < frames.Length; f++)
                {
                    frames[f] = new Frame { Index = reader.ReadInt32(), Positions = new float3[bodies], Rotations = new float4[bodies] };
                    for (var b = 0; b < bodies; b++)
                    {
                        frames[f].Positions[b] = new float3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                        frames[f].Rotations[b] = new float4(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                    }
                }

                _reference[id] = frames;
            }

            return _reference;
        }
    }
}
