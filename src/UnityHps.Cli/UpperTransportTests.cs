using System.Numerics;
using UnityHps;

internal static class UpperTransportTests
{
    public static void Run(Action<bool,string> check)
    {
        Rig source=OfflineRigFactory.LoadProfile(false);
        var target=new Rig{bones=source.bindBones.Select(b=>new Bone{name=b.name,parent=b.parent,humanBone=b.humanBone,
            position=(float[])b.position.Clone(),rotation=(float[])b.rotation.Clone()}).ToArray()};
        // Different rest arm elevations make the old per-segment conjugations
        // disagree even when the source only turns as one rigid body.
        foreach(var bone in target.bones.Where(b=>b.humanBone is 13 or 14))
            bone.rotation=Math3.A(Quaternion.Normalize(Math3.Q(bone.rotation)*Quaternion.CreateFromAxisAngle(Vector3.UnitZ,bone.humanBone==13?.35f:-.35f)));
        var bind=new Rig{bones=source.bindBones};
        var adapter=new SourcePoseAdapter(source,bind,target);
        SkeletonPose Map(SkeletonPose pose)
        {
            var rotations=adapter.Map(pose.ToFrame());
            var frame=new SkeletonPose(target).ToFrame();frame.rotations=rotations.Select(x=>(float)x).ToArray();
            return new SkeletonPose(target,frame);
        }
        var rest=new SkeletonPose(bind);var mappedRest=Map(rest);var targetRest=new SkeletonPose(target);
        check(Enumerable.Range(0,target.bones.Length).All(i=>Math3.Angle(mappedRest.WorldRotations[i],targetRest.WorldRotations[i])<.002f),
            "upper transport maps the source rest to the target rest");
        var turn=new SkeletonPose(bind);var rotation=Quaternion.CreateFromYawPitchRoll(.8f,-.3f,.2f);
        for(int i=0;i<bind.bones.Length;i++)if(bind.bones[i].parent<0)turn.LocalRotations[i]=Quaternion.Normalize(rotation*turn.LocalRotations[i]);
        turn.UpdateWorld();var mapped=Map(turn);
        check(Enumerable.Range(0,target.bones.Length).Where(i=>target.bones[i].humanBone is >=11 and <=18)
            .All(i=>Math3.Angle(mapped.LocalRotations[i],mappedRest.LocalRotations[i])<.002f),
            "a rigid source turn does not invent relative shoulder, elbow or hand motion across different rest arms");
        check(Enumerable.Range(0,target.bones.Length).Where(i=>target.bones[i].humanBone is >=11 and <=18)
            .All(i=>Math3.Angle(mapped.WorldRotations[i],Quaternion.Normalize(rotation*targetRest.WorldRotations[i]))<.002f),
            "all transported arm segments retain the same rigid body rotation");
        int elbow=Array.FindIndex(bind.bones,b=>b.humanBone==15);
        var bent=new SkeletonPose(bind);
        bent.LocalRotations[elbow]=Quaternion.Normalize(bent.LocalRotations[elbow]*Quaternion.CreateFromAxisAngle(Vector3.UnitZ,.6f));
        bent.UpdateWorld();var mappedBent=Map(bent);
        var bentTurn=new SkeletonPose(bind,bent.ToFrame());
        for(int i=0;i<bind.bones.Length;i++)if(bind.bones[i].parent<0)bentTurn.LocalRotations[i]=Quaternion.Normalize(rotation*bentTurn.LocalRotations[i]);
        bentTurn.UpdateWorld();var mappedBentTurn=Map(bentTurn);
        check(Enumerable.Range(0,target.bones.Length).Where(i=>target.bones[i].humanBone is >=11 and <=18)
            .All(i=>Math3.Angle(mappedBentTurn.LocalRotations[i],mappedBent.LocalRotations[i])<.002f),
            "bending an elbow while turning retains the same relative arm articulation");
        check(Math3.Angle(mappedBent.WorldRotations[elbow],mappedRest.WorldRotations[elbow])>30,
            "common-frame transport retains elbow articulation instead of freezing the chain");
    }
}
