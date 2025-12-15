using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;


namespace g3
{
    public static class ShadowGenerator
    {
        /// <summary>
        /// Compute the shadow (projection onto x-y-plane) of the given mesh using g3sharp edge loops.
        /// </summary>
        /// <param name="mesh">input mesh</param>
        /// <param name="minArea">area size filter for shadow polygons</param>
        /// <returns></returns>
        public static GeneralPolygon2d[] GetShadowPolygons2d(DMesh3 mesh, double minArea = 0.01)
        {
            var shadows = UpskinBoundaryLoopsPolys(mesh)
                   .Where(poly => poly.Area > minArea)
                   .Select(poly => new GeneralPolygon2d(poly));
            return PostProcessShadows(mesh, minArea, shadows);
        }

        private static GeneralPolygon2d[] PostProcessShadows(DMesh3 mesh, double minArea, IEnumerable<GeneralPolygon2d> input)
        {
            IEnumerable<GeneralPolygon2d> shadows = input;
            if (mesh.cutShadowHint != null)
            {
                shadows = input.Concat(mesh.cutShadowHint);
            }
            if (!shadows.Any())
                throw new System.ApplicationException("shadow compute failed. No contours found");

            //get the union, this contains inner "holes" as well
            var union = shadows.Union();
            var result = union.FilterMinArea(minArea).OrderByDescending(x => x.Area).ToArray();
            return result;
        }

        public static IEnumerable<Polygon2d> PiecewiseConvexShadow(DMesh3 mesh, int elementsPerBatch, double minArea = 0.01)
        {
            var dmesh = mesh;
            var meshAssembly = new MeshConnectedComponents(dmesh);

            meshAssembly.FilterF = dmesh.TriIsUpSkin;
            meshAssembly.FindConnectedT();
            var bag = new ConcurrentBag<Polygon2d>();
            foreach (var component in meshAssembly)
            {
                // sort by x of first triangle vertex using low level buffers
                var vertices = dmesh.VerticesBuffer;
                var triangles = dmesh.TrianglesBuffer;
                component.Indices.SortBy(i => vertices[triangles[i * 3] * 3]);
                int batchCount = (int)Math.Ceiling((double)component.Indices.Length / elementsPerBatch);
                if (batchCount < Environment.ProcessorCount / 2)
                {
                    foreach (var idx in component.Indices)
                    {
                        bag.Add(MakeHull(dmesh, component, 0, component.Indices.Length));
                    }
                }
                else
                {
                    System.Threading.Tasks.Parallel.For(0, batchCount, batchIdx =>
                    {
                        int start = elementsPerBatch * batchIdx;
                        int end = Math.Min(start + elementsPerBatch, component.Indices.Length);
                        Polygon2d hull = MakeHull(dmesh, component, start, end);
                        bag.Add(hull);
                    });
                }
            }
            return bag;
        }

        private static Polygon2d MakeHull(DMesh3 dmesh, MeshConnectedComponents.Component component, int start, int end)
        {
            var pool = System.Buffers.ArrayPool<Vector2d>.Shared;
            int size = (end - start) * 3;
            var buffer = pool.Rent(size);
            ArraySegment<Vector2d> projectedVertices = new ArraySegment<Vector2d>(buffer, 0, size);
            Vector2d v0 = new(), v1 = new(), v2 = new();
            for (int i = start; i < end; i++)
            {
                int triID = component.Indices[i];
                dmesh.GetTriVertices2D(triID, ref v0, ref v1, ref v2);
                projectedVertices[i] = v0;
                projectedVertices[i + 1] = v1;
                projectedVertices[i + 2] = v2;
            }
            var hull = projectedVertices.ConvexHull();
            pool.Return(buffer);
            hull.Simplify();
            return hull;
        }


        public static void AddCutToG3Mesh(DMesh3 mesh, g3.MeshPlaneCut cut)
        {
            var polysToSort = new List<Polygon2d>();
            if (cut.CutLoops != null)
                foreach (var loop in cut.CutLoops)
                    polysToSort.Add(TypeConversions.PolygonFromVtxIndices(cut.Mesh, loop.Vertices));
            if (cut.CutSpans != null)
                foreach (var span in cut.CutSpans)
                    polysToSort.Add(TypeConversions.PolygonFromVtxIndices(cut.Mesh, span.Vertices));
            if (polysToSort.Count > 0)
            {
                var sorted = polysToSort.SortPolygons();

                if (mesh.cutShadowHint == null)
                {
                    mesh.cutShadowHint = sorted;
                }
                else
                {
                    var union = mesh.cutShadowHint.Concat(sorted).Union();
                    mesh.cutShadowHint = union;
                }
            }
        }

        private static IEnumerable<Polygon2d> UpskinBoundaryLoopsPolys(DMesh3 mesh)
        {
            //get upskin mesh
            var upSkin = mesh.FilteredTrianglesSubmesh(DMesh3Extensions.TriIsUpSkin, MeshComponents.None);
            return MeshBoundaryLoopsPolys(upSkin);
        }

        private static IEnumerable<Polygon2d> MeshBoundaryLoopsPolys(DMesh3 mesh)
        {
            if (mesh.TriangleCount <= 0) yield break;
            MeshBoundaryLoops computeEdgeLoops = new MeshBoundaryLoops(mesh);

            //extract edge loops, these are possible contour edges of shadows
            computeEdgeLoops.Compute();

            foreach (var edgeLoop in computeEdgeLoops.Loops)
            {
                yield return TypeConversions.PolygonFromVtxIndices(computeEdgeLoops.Mesh, edgeLoop.Vertices);
            }

            //we use spans anyways (!and close them by connecting start to end), this might create chords that enlarge the shadow
            //but can never be worse than the convex shadow/hull
            //the resulting (self-) intersections should be fixed with a union
            foreach (var span in computeEdgeLoops.Spans)
            {
                var polyline = TypeConversions.PolygonFromVtxIndices(computeEdgeLoops.Mesh, span.Vertices);
                yield return polyline;
            }
        }

        public static IEnumerable<Polygon2d> GetShadowPolygons2dOpenMesh(DMesh3 mesh, double lineThickness)
        {
            var shadows = new List<Polygon2d>();
            //if (mesh.cutShadowHint != null) shadows.AddRange(mesh.cutShadowHint);
            foreach (var decomp in OpenMeshDecompose.ConnectedComponentsAs2D(mesh, lineThickness))
            {
                if (decomp.dimension == 2)
                {
                    //for open meshes we first don't filter for upskin, we use the open edges of each component
                    //this will *not* work if the open mesh is actually a mesh that should be closed but has holes
                    //since we then only extract the hole edges as shadows
                    //so we filter for upskin as well and add those edge loops on top
                    //union (hopefully) fixes everything afterwards
                    var compMesh = mesh.QuickSubmesh(decomp.triangleIndices, MeshComponents.None, decomp.triangleIndices.Length);

                    var openEdges = MeshBoundaryLoopsPolys(compMesh);
                    var upskinEdges = UpskinBoundaryLoopsPolys(compMesh);
                    var allEdges = openEdges.Concat(upskinEdges);

                    foreach (var poly in allEdges)
                    {
                        //poly.Simplify();
                        foreach (var offset in poly.OffsetFastUnsafe(lineThickness / 2))
                        {
                            if (offset.IsHole) offset.Reverse();
                            shadows.Add(offset);
                        }
                    }
                }
                else if (decomp.dimension == 1)
                {
                    var box = decomp.OneDimBounds.Shadow();
                    Debug.Assert(!box.IsHole);
                    shadows.Add(box);
                }
                else if (decomp.dimension == 0)
                {
                    shadows.Add(decomp.vertices2d.Bounds().ToPolygon2d());
                }
            }

            if (shadows.Count == 0) throw new System.ApplicationException("shadow compute failed. No contours found");
            //get the union, this contains inner "holes" as well
            //var union = shadows.SortPolygons(Clipper2Lib.FillRule.NonZero);
            return shadows;
        }

        public const double MinimumLineThickness = 0.001;

        public static IList<GeneralPolygon2d> ComputeMeshPartitionShadow(MeshZPartition partition, int triangleCountLimit = 1_000_000, double aggregationDistance = 0, double lineThickness = MinimumLineThickness, int debugSVGLayerNr = int.MinValue)
        {
            double zEnd = partition.zEnd;
            DMesh3 meshSlice = partition.MakeSubmesh(MeshComponents.None);

            if (meshSlice.TriangleCount == 0)
                return Array.Empty<GeneralPolygon2d>();

            meshSlice.PartName = partition.PartitionName;

            //we don't need to fill holes for shadow computation
            meshSlice.ZParallelDoubleCut(partition.zStart, zEnd, fillHoles: false);
            if (meshSlice.TriangleCount == 0) { return Array.Empty<GeneralPolygon2d>(); }

            //we do it this way to avoid dependency to G3SharpMesh in MeshZPartition.
            //note we can't differentiate between PART and SOLIDSUPPORT, but also don't need to
            meshSlice.MeshType = partition.mesh.CachedIsClosed ? MeshType.PART : MeshType.SUPPORT;

            double minArea = 0.01;
            //these are not saved in the cache
            IEnumerable<GeneralPolygon2d> shadows;
            //the MeshBoundaryLoops algorithm in g3 is very slow for big meshes
            if (meshSlice.TriangleCount > triangleCountLimit)
            {
                shadows = PiecewiseConvexShadow(meshSlice, triangleCountLimit / 100).Select(x => new GeneralPolygon2d(x));
            }
            else if (meshSlice.MeshType == MeshType.SUPPORT)
            {
                var thickness = Math.Min(lineThickness, MinimumLineThickness);
                shadows = GetShadowPolygons2dOpenMesh(meshSlice, thickness).Select(x => new GeneralPolygon2d(x));
            }
            else
            {
                // filter shadows of closed meshes unconditionally by area
                // this is much faster for aggregate and for closed meshes small polys are always artifacts
                shadows = UpskinBoundaryLoopsPolys(meshSlice)
                   .Where(poly => poly.Area > minArea).SortPolygons(fillRule: Clipper2Lib.FillRule.NonZero);

                if (meshSlice.cutShadowHint != null)
                {
                    // it is important to keep the cutShadowHint intact for batched aggregate because it has polygon holes
                    shadows = shadows.Concat(meshSlice.cutShadowHint);
                }
            }

            var cleanedShadows = CleanDegenerateDimensions(shadows, minArea);

            if (aggregationDistance > 0)
            {
                //aggregate does simplify, so we do not here
                var aggregation = cleanedShadows.AggregateBatched(AggregationBatchSize, aggregationDistance, allowVertexIncreasingCords: false, arcTolerance: 0.4);
                return aggregation;
            }
            else
            {
                foreach (var shad in cleanedShadows) shad.Simplify();
                return cleanedShadows.Union();
            }
        }

        private static List<GeneralPolygon2d> CleanDegenerateDimensions(IEnumerable<GeneralPolygon2d> polys, double minArea)
        {
            var filteredPolys = new List<GeneralPolygon2d>();
            foreach (var poly in polys)
            {
                if (poly.Area >= minArea)
                {
                    var holes = poly.Holes;
                    if (holes.Any(h => h.Area < minArea))
                    {
                        var bigHoles = holes.Where(h => h.Area >= minArea).ToArray();
                        poly.ClearHoles();
                        foreach (var hole in bigHoles) { poly.AddHole(hole, false, false); }
                    }
                    filteredPolys.Add(poly);
                }
                else
                {
                    var minBounds = poly.Outer.MinimalBoundingBox();
                    double minExt = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE;
                    if (minBounds.Extent.x < minExt) { minBounds.Extent.x = minExt; }
                    if (minBounds.Extent.y < minExt) { minBounds.Extent.y = minExt; }
                    filteredPolys.Add(new GeneralPolygon2d(minBounds.ToPolygon2d()));
                }
            }
            return filteredPolys;
        }

        public static int AggregationBatchSize = 400;
        public static int AggregationBatchMinCount = 25;

        private static List<GeneralPolygon2d> AggregateBatched(this IEnumerable<GeneralPolygon2d> polyEnum, int batchSize, double aggregationTolerance, bool allowVertexIncreasingCords = false, double arcTolerance = 0.1, double miterLimit = 2.0)
        {
            if (AggregationBatchMinCount <= 0 || polyEnum.Count() < AggregationBatchMinCount)
            {
                return polyEnum.Aggregate(aggregationTolerance, arcTolerance, miterLimit, allowVertexIncreasingCords, true);
            }

            var polys = polyEnum.ToArray();
            polys.SortByDescending(x => x.Area + x.Outer.Vertices[0].DistanceSquared(new Vector2d(-100000, -100000)));
            int vtxCount = 0;
            const int minAggrCount = 3;
            List<GeneralPolygon2d> buffer = new();
            List<GeneralPolygon2d> intermediateResult = new();
            for (int i = 0; i < polys.Length; i++)
            {
                var poly = polys[i];
                //do not process large polys more than once
                if (poly.VertexCount > batchSize)
                {
                    intermediateResult.Add(poly);
                    continue;
                }

                buffer.Add(poly);
                vtxCount += poly.VertexCount;

                if (vtxCount >= batchSize && buffer.Count >= minAggrCount)
                {
                    var batchResult = buffer.Aggregate(aggregationTolerance, arcTolerance, miterLimit, allowVertexIncreasingCords, true);
                    intermediateResult.AddRange(batchResult);
                    buffer.Clear();
                    vtxCount = 0;
                }
            }
            intermediateResult.AddRange(buffer);

            return intermediateResult.Aggregate(aggregationTolerance, arcTolerance, miterLimit, allowVertexIncreasingCords, true);
        }
    }
}
