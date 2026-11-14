using UnityHps;

internal static class UpperProjectionTests
{
    public static void Run(Action<bool, string> check)
    {
        Rig rig = OfflineRigFactory.LoadProfile(false);
        var projector = new HumanoidUpperProjector(rig);
        PoseFrame frame = new HumanoidSolver(rig).Decode(new float[rig.muscles.Length]).ToFrame();
        float[] seed = Enumerable.Repeat(.01f, rig.muscles.Length).ToArray();
        float[] result = projector.Project(frame, seed);
        check(result.All(float.IsFinite) && !ReferenceEquals(result, seed) && seed.All(x => x == .01f) &&
            Enumerable.Range(0, seed.Length).Where(i => rig.muscles[i].unityIndex is < 37 or > 54).All(i => result[i] == seed[i]),
            "embedded upper projector owns output and preserves non-upper channels");
        Parallel.For(0, 8, _ =>
        {
            if (!projector.Project(frame, seed).SequenceEqual(result))
                throw new Exception("Shared upper response model changed under parallel calls.");
        });
        check(true, "embedded upper response is deterministic under parallel projection");
        var decoder=new HumanoidSolver(rig,rangePolicy:HumanoidRangePolicy.Tangent);
        var input=Enumerable.Range(0,13).Select(i=>{
            var v=new float[rig.muscles.Length];v[41]=i<6?1.96f:-1.96f;v[42]=.65f;v[43]=.4f;
            return v;
        }).ToArray();
        var motion=input.Select((v,i)=>decoder.Decode(v).ToFrame(i/120f)).ToArray();
        var repaired=projector.StabilizeBatch(motion,input,1,rangePolicy:HumanoidRangePolicy.Tangent);
        int[] segments=rig.bones.Select((b,i)=>(b,i)).Where(x=>x.b.humanBone is 13 or 15).Select(x=>x.i).ToArray();
        double worst=0;
        for(int i=0;i<motion.Length;i++){
            var source=new SkeletonPose(rig,motion[i]);var actual=decoder.Decode(repaired[i]);
            foreach(int b in segments)worst=Math.Max(worst,Math3.Angle(source.WorldRotations[b],actual.WorldRotations[b]));
        }
        check(worst<2.1,"upper continuity does not trade known segment orientation for endpoint continuity");
        check(input.All(v=>v[42]==.65f&&v[43]==.4f)&&repaired.Select((v,i)=>Enumerable.Range(0,v.Length).Where(c=>rig.muscles[c].unityIndex is <37 or >54).All(c=>v[c]==input[i][c])).All(x=>x),
            "upper continuity keeps caller values and non-upper channels unchanged");
        var projected=motion.Select((f,i)=>projector.Project(f,input[i])).ToArray();
        var continued=projector.ProjectSourceBatch(motion,input,1,rangePolicy:HumanoidRangePolicy.Tangent);
        double regression=0;
        for(int i=0;i<motion.Length;i++){
            var source=new SkeletonPose(rig,motion[i]);var before=decoder.Decode(projected[i]);var after=decoder.Decode(continued[i]);
            foreach(int b in segments)regression=Math.Max(regression,Math3.Angle(source.WorldRotations[b],after.WorldRotations[b])-Math3.Angle(source.WorldRotations[b],before.WorldRotations[b]));
        }
        check(regression<2.1,"source branch continuation preserves current physical segment orientation");
        var reference=projector.ReferenceModel;
        Q Roll(double degrees)=>new(Math.Sin(degrees*Math.PI/360),0,0,Math.Cos(degrees*Math.PI/360));
        Q identity=new(0,0,0,1);
        Q[] physical=[identity,identity,identity,identity];
        check(!UpperPoseContinuation.PreservesSegmentPose(
            [identity,Roll(20),Roll(-20),identity],
            [identity,Roll(21.5),Roll(-21.5),identity],physical),
            "segment guard rejects increased elbow-relative twist hidden inside individual orientation allowances");
        check(!UpperPoseContinuation.PreservesSegmentPose(
            [identity,identity,Roll(20),Roll(-20)],
            [identity,identity,Roll(21.5),Roll(-21.5)],physical),
            "segment guard rejects increased wrist-relative twist hidden inside individual orientation allowances");
        check(UpperPoseContinuation.PreservesSegmentPose(
            [identity,Roll(20),Roll(20),Roll(20)],
            [identity,Roll(21),Roll(21),Roll(21)],physical),
            "relative twist guard permits a common arm rotation within the existing orientation budget");
        check(UpperPoseContinuation.PreservesSegmentPose(
            [identity,Roll(40),Roll(-40),Roll(-40)],
            [identity,Roll(30),Roll(-30),Roll(-30)],physical),
            "relative twist guard permits improved segment and joint orientation even when the baseline is distorted");
        var guard=new UpperPoseContinuation(reference,Enumerable.Repeat(540d,18).ToArray(),Enumerable.Repeat(540d,18).ToArray());
        double[][] shortEdge=[
            [0,0,1.7631581442,-1.7010310990,-1.0621141710,.1495401691,1.7121873702,.8580527685,-1.6020881216,0,0,0,0,0,0,0,0,0],
            [0,0,-1.6987214790,-.4795557215,.6423595492,-1.6442241081,.5559935286,.1675777381,-.1581594897,0,0,0,0,0,0,0,0,0]];
        // A baseline can have a concentrated spike with less total travel.
        // Spreading a near-full turn across the edge lowers the largest step
        // but still adds unwanted motion and must be rejected.
        var longEdge=new double[][]{new double[18],new double[18]};
        var swingMuscle=reference.ReferenceRig.muscles[39];
        longEdge[0][2]=175/swingMuscle.maximum;longEdge[1][2]=-175/-swingMuscle.minimum;
        check(guard.ValidateRepairRuns(shortEdge,longEdge)==1&&longEdge[1][2]==shortEdge[1][2],
            "continuation rejects extra total rotation even when the largest interior step decreases");
        var stationary=Enumerable.Range(0,361).Select(_=>new double[18]).ToArray();
        var slowTurn=stationary.Select((a,i)=>{
            var v=(double[])a.Clone();v[2]=i/swingMuscle.maximum;return v;
        }).ToArray();
        check(guard.ValidateRepairRuns(stationary,slowTurn)==1&&slowTurn.SelectMany(a=>a).All(x=>x==0),
            "continuation rejects a slow full turn spread across many small frame steps");
        var bothSides=stationary.Select((a,i)=>{
            var v=(double[])a.Clone();v[11]=i/reference.ReferenceRig.muscles[48].maximum;
            if(i==180)v[2]=.001;return v;
        }).ToArray();
        check(guard.ValidateRepairRuns(stationary,bothSides)==1&&bothSides[180][2]==.001&&
            bothSides.All(a=>a[11]==0)&&stationary.SelectMany(a=>a).All(x=>x==0),
            "travel guard rolls back only the rejected arm and preserves the baseline input");
        var smallOriginal=new[]{new double[18],new double[18],new double[18]};
        var smallCorrection=smallOriginal.Select(a=>(double[])a.Clone()).ToArray();
        smallCorrection[1][2]=.001;
        check(guard.ValidateRepairRuns(smallOriginal,smallCorrection)==0&&smallCorrection[1][2]==.001,
            "travel guard retains a small correction within pose and travel budgets");
        var equalEdge=shortEdge.Select(a=>(double[])a.Clone()).ToArray();
        check(guard.ValidateRepairRuns(shortEdge,equalEdge)==0,"travel guard preserves unchanged input motion including existing spikes");
        check(Math.Abs(SourceUpperPoseContinuation.Principal(-179.99995)+179.99995)<1e-9,
            "source twist normalization preserves the negative side immediately inside the native cut");
        check(SourceUpperPoseContinuation.Principal(180)==180&&SourceUpperPoseContinuation.Principal(-180)==-180&&
            SourceUpperPoseContinuation.Principal(540)==180&&SourceUpperPoseContinuation.Principal(-540)==-180,
            "source twist normalization retains exact endpoint signs used by native splitting");
        check(Math.Abs(SourceUpperPoseContinuation.Principal(180.0002)+179.9998)<1e-9&&
            Math.Abs(SourceUpperPoseContinuation.Principal(-180.0002)-179.9998)<1e-9,
            "source twist normalization retains the side of values beyond either cut");
        foreach(int slot in new[]{4,6,13,15}){
            var muscle=reference.ReferenceRig.muscles[37+slot];
            foreach(double degrees in new[]{-179.99995,-180d,180d}){
                var value=new double[18];
                value[slot]=degrees/(degrees<0?-muscle.minimum:muscle.maximum);
                var expected=(double[])value.Clone();
                var raw=reference.ReferenceRig.bones.SelectMany(b=>b.rotation.Select(x=>(double)x)).ToArray();
                guard.StabilizeSequence([raw],[value]);
                check(Math.Abs(value[slot]-expected[slot])<1e-12&&
                    Enumerable.Range(0,18).Where(c=>c!=slot).All(c=>value[c]==expected[c]),
                    $"target continuation preserves native twist branch at channel {37+slot}, degrees {degrees}");
            }
        }
        var sourceGuard=new SourceUpperPoseContinuation(reference,Enumerable.Repeat(540d,18).ToArray(),Enumerable.Repeat(540d,18).ToArray());
        var jointByHuman=reference.Joints.ToDictionary(j=>j.HumanBone);
        foreach(int slot in new[]{4,6,13,15}){
            int side=slot/9,owner=slot%9==4?1:2;
            foreach(double degrees in new[]{-540,-180.0002,-180d,-179.99995,179.99995,180d,180.0002,540}){
                var values=new double[18];var muscle=reference.ReferenceRig.muscles[37+slot];
                values[slot]=degrees/(degrees<0?-muscle.minimum:muscle.maximum);
                var neutral=new double[18];
                // Independent expected half-twist, including the inherited rotation
                // in the adjacent child. Check the actual guard decoder, not only
                // its serialized muscle normalization.
                float phase=(float)(degrees*Math.PI/180*jointByHuman[13+side+(owner-1)*2].LimitSign[0]);
                Q half=Roll(MathF.Atan2(MathF.Sin(phase),MathF.Cos(phase))*90/Math.PI);
                Q[] Expected(Q[] zero){
                    var result=(Q[])zero.Clone();int human=11+side+owner*2;
                    Q ownPost=Q.Read(jointByHuman[human].Post);
                    result[owner]=result[owner]*ownPost*half*ownPost.Inverse;
                    result[owner+1]=ownPost*half*ownPost.Inverse*result[owner+1];return result;
                }
                double Error(Q[] expected,Q[] actual)=>expected.Zip(actual,(a,b)=>{
                    Q q=(a.Inverse*b).Normalized;return 2*Math.Atan2(Math.Sqrt(q.X*q.X+q.Y*q.Y+q.Z*q.Z),Math.Abs(q.W))*180/Math.PI;
                }).Max();
                check(Error(Expected(guard.DecodeLocal(neutral,side)),guard.DecodeLocal(values,side))<.0001&&
                    Error(Expected(sourceGuard.DecodeLocal(neutral,side)),sourceGuard.DecodeLocal(values,side))<.0001,
                    $"source and target guard FK retain signed native twist halves at channel {37+slot}, degrees {degrees}");
            }
        }

    }
}
