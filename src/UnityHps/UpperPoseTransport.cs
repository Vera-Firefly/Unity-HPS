using System.Numerics;
using UnityHps;
using NQ = System.Numerics.Quaternion;

namespace UnityHps;

// Model-independent frame transport into the candidate's reference joint coordinates.
// This is an approximate anatomical transfer, not per-model native calibration.
sealed class SourcePoseAdapter
{
    readonly Rig source, reference;
    readonly SkeletonPose referenceBind;
    readonly Dictionary<int, int> sourceByHuman;
    readonly Dictionary<int, NQ> alignment;
    readonly NQ[] restWorld;

    public SourcePoseAdapter(Rig rig, Rig bindPose, Rig referencePose)
    {
        source = rig; reference = referencePose;
        referenceBind = new SkeletonPose(reference);
        sourceByHuman = source.bones.Select((b, i) => (b, i)).Where(x => x.b.humanBone >= 0).ToDictionary(x => x.b.humanBone, x => x.i);
        foreach (int h in new[] { 1, 2, 13, 14, 15, 16, 17, 18 })
            if (!sourceByHuman.ContainsKey(h)) throw new InvalidDataException("Missing Humanoid transport bone: " + HumanBones.Names[h]);
        if (bindPose.bones.Length != rig.bones.Length) throw new InvalidDataException("Bind pose bone count differs");
        for (int i = 0; i < rig.bones.Length; i++)
            if (bindPose.bones[i].parent != rig.bones[i].parent || bindPose.bones[i].humanBone != rig.bones[i].humanBone)
                throw new InvalidDataException("Bind pose layout differs");
        var sourceBind = new SkeletonPose(bindPose);
        restWorld = sourceBind.WorldRotations;
        alignment = Align(sourceBind, referenceBind, sourceByHuman);
    }

    Dictionary<int, NQ> Align(SkeletonPose sourceBind, SkeletonPose referenceBind, Dictionary<int, int> sourceByHuman)
    {
        var referenceByHuman = reference.bones.Select((b, i) => (b, i)).Where(x => x.b.humanBone >= 0).ToDictionary(x => x.b.humanBone, x => x.i);
        NQ Body(SkeletonPose p, Dictionary<int, int> by)
        {
            Vector3 Get(int h) => p.WorldPositions[by[h]];
            Vector3 up = (Get(13) + Get(14) - Get(1) - Get(2)) * .5f;
            Vector3 right = Get(14) - Get(13) + Get(2) - Get(1);
            return Math3.LookRotation(Vector3.Cross(right, up), up);
        }
        NQ sourceBody = Body(sourceBind, sourceByHuman), referenceBody = Body(referenceBind, referenceByHuman);
        // A motion delta lives in one world frame. Conjugating that same delta
        // separately by each segment's rest direction turns a rigid body motion
        // into relative joint motion. Rest rotations already retain the target
        // arm elevation and joint axes; only the body coordinate frame differs.
        NQ bodyAlignment=NQ.Normalize(referenceBody*NQ.Inverse(sourceBody));
        var alignment = new Dictionary<int, NQ>();
        foreach (int h in referenceByHuman.Keys.Where(sourceByHuman.ContainsKey))
            alignment[h] = bodyAlignment;
        return alignment;
    }

    public double[] Map(PoseFrame frame)
    {
        var pose = new SkeletonPose(source, frame);
        var output = new SkeletonPose(reference);
        for (int i = 0; i < reference.bones.Length; i++)
        {
            var b = reference.bones[i];
            if (sourceByHuman.TryGetValue(b.humanBone, out int sourceBone))
            {
                NQ a = alignment[b.humanBone];
                NQ delta = NQ.Normalize(pose.WorldRotations[sourceBone] * NQ.Inverse(restWorld[sourceBone]));
                NQ world = NQ.Normalize(a * delta * NQ.Inverse(a) * referenceBind.WorldRotations[i]);
                output.SetWorldRotation(i, world);
            }
        }
        return output.ToFrame(frame.time).rotations.Select(x => (double)x).ToArray();
    }
}
