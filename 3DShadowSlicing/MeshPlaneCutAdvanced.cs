using g3;

namespace g3
{
    public static class MeshPlaneCutAdvanced
    {
        public static bool AdvancedHoleFill(MeshPlaneCut cut)
        {
            bool success = cut.FillHoles();
            if (!success)
            {
                //revert fill
                foreach (var loop in cut.LoopFillTriangles)
                    foreach (var idx in loop)
                        cut.Mesh.RemoveTriangle(idx);

                var filler = new PlanarHoleFiller(cut.Mesh);
                filler.AddFillLoops(cut.CutLoops);
                filler.SetPlane(cut.PlaneOrigin, cut.PlaneNormal);
                if (cut.CutSpans != null)
                {
                    foreach (var span in cut.CutSpans)
                    {
                        EdgeLoop loop = new EdgeLoop(cut.Mesh, span.Vertices, span.Edges, true);
                        //loop.CheckValidity();
                        filler.AddFillLoop(loop);
                    }
                }
                try { success = filler.Fill(); }
                catch { }
                //try to repair cracks and degenerate edges
                var merge = new MergeCoincidentEdges(cut.Mesh);
                merge.OnlyUniquePairs = true;
                merge.MergeDistance = 0.0001;
                success = merge.Apply();
            }
            return cut.Mesh.CachedIsClosed;
        }
    }
}
