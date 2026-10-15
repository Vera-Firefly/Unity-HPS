using System.Numerics;
using UnityHps;

internal static class SelfTests
{
    public static void Run()
    {
        int passed = 0;
        void Check(bool condition, string name)
        { if (!condition) throw new InvalidOperationException("FAIL: " + name); Console.WriteLine("PASS: " + name); passed++; }
        Rig rig = Demo.CreateRig(); var solver = new HumanoidSolver(rig);
        PoseBatch poses = Demo.CreatePoses(rig, 24);
        ResultBatch result = solver.SolveBatch(poses, 1);
        AccuracyReport report = solver.Compare(poses, result);
        Check(report.maxRootPositionMeters < 1e-5f && report.maxRootRotationDegrees < 0.001f && report.maxMuscleError < 0.0001f, "synthetic 3-axis + moving/rotating root round trip");
        ResultBatch parallel = solver.SolveBatch(poses, 4);
        Check(result.frames.Zip(parallel.frames).All(pair => pair.First.humanPose.muscles.SequenceEqual(pair.Second.humanPose.muscles)), "parallel determinism");
        var skeleton = new SkeletonPose(rig);
        float error = TwoBoneIk.Solve(skeleton, new IkGoal { root = 3, middle = 4, tip = 5, target = [-0.12f, 0.3f, 0.15f], pole = [-0.12f, 0.4f, 1] });
        Check(error < 0.00001f, "reachable two-bone IK end position");
        Check(Math.Abs(Vector3.Distance(skeleton.WorldPositions[3], skeleton.WorldPositions[4]) - 0.45f) < 1e-6f &&
            Math.Abs(Vector3.Distance(skeleton.WorldPositions[4], skeleton.WorldPositions[5]) - 0.45f) < 1e-6f, "IK preserves both limb lengths");
        Quaternion tipBefore = skeleton.WorldRotations[5];
        error = TwoBoneIk.Solve(skeleton, new IkGoal { root = 3, middle = 4, tip = 5, target = [-0.12f, -4, 0], pole = [-0.12f, -4, 0] });
        Check(float.IsFinite(error) && error > 4, "unreachable/collinear IK stays finite without stretching");
        Check(Math3.Angle(tipBefore, skeleton.WorldRotations[5]) < 0.001f, "IK preserves original tip orientation when unspecified");
        PoseFrame bad = new() { positions = [0, 0, 0], rotations = [0, 0, 0, 1] };
        bool rejected = false;
        try { solver.Solve(bad); } catch (InvalidDataException) { rejected = true; }
        Check(rejected, "reject incomplete pose instead of silently defaulting bones");
        rejected = false;
        try { solver.Decode([float.NaN, 0, 0]); } catch (InvalidDataException) { rejected = true; }
        Check(rejected, "reject non-finite muscles");
        Check(Math3.Angle(Quaternion.Identity, Math3.FromTo(Vector3.UnitY, -Vector3.UnitY)) > 179.9f, "antiparallel quaternion handling");
        // Rotation-invariant RootT affine mapping in world axes.
        Rig rootRig = Demo.CreateRig(); rootRig.bodyPositionJacobian = new float[rootRig.bones.Length * 9];
        rootRig.bodyPositionJacobian[0] = rootRig.bodyPositionJacobian[4] = rootRig.bodyPositionJacobian[8] = 1;
        var rootSolver = new HumanoidSolver(rootRig);
        report = rootSolver.Compare(poses, rootSolver.SolveBatch(poses, 1));
        Check(report.maxRootPositionMeters < 1e-5f, "RootT Jacobian uses world-space position once");
        Rig nested = Demo.CreateRig();
        foreach (var b in nested.bones) b.parent++;
        nested.bones = new[] { new Bone { name = "AnimatorRoot" } }.Concat(nested.bones).ToArray();
        nested.hips++;
        foreach (var group in nested.groups) group.bones = group.bones.Select(b => b + 1).ToArray();
        var nestedSolver = new HumanoidSolver(nested);
        var nestedPoses = Demo.CreatePoses(nested, 12);
        report = nestedSolver.Compare(nestedPoses, nestedSolver.SolveBatch(nestedPoses, 2));
        Check(report.maxRootPositionMeters < 1e-5f && nestedPoses.frames.All(f => Math3.V(f.positions) == Vector3.Zero && Math3.Angle(Math3.Q(f.rotations), Quaternion.Identity) < 0.001f),
            "HumanPose moves Hips under an unchanged Animator root");
        var analytic = new SkeletonPose(rig);
        analytic.LocalRotations[1] = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.23f * 0.6f) *
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -0.31f * 0.5f) * Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.62f * 0.4f);
        analytic.UpdateWorld();
        HumanPose fitted = solver.Solve(analytic.ToFrame()).humanPose;
        Check(Math.Abs(fitted.muscles[0] - 0.23f) < 0.001f && Math.Abs(fitted.muscles[1] + 0.31f) < 0.001f && Math.Abs(fitted.muscles[2] - 0.62f) < 0.001f,
            "inverse fit against independent analytic quaternion pose");
        Console.WriteLine($"{passed} tests passed. These synthetic tests do NOT establish Unity equivalence.");
    }
}
