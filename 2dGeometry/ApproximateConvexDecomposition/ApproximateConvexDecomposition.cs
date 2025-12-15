using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using static g3.ConvexHullBridgeFinder;
using BridgeResult = g3.ConvexHullBridgeFinder.BridgeResult<g3.ApproximateConvexDecomposition.ConcavityMeasureBridge>;

namespace g3
{
    public static partial class ApproximateConvexDecomposition
    {
        public enum ConcavityMeasure
        {
            /// <summary>
            /// Find the nearest bridge of the convex hull going in a straigth line. Concavity is defined as the length of the line.
            /// </summary>
            StraightLineMeasurement = 0,
            /// <summary>
            /// Find the shorest path INSIDE the polygon pocket to the convex hull. Concavity is defined as the length of the path.
            /// </summary>
            ShortestPathMeasurement = 1,
            /// <summary>
            /// check if SP is needed before using SL
            /// </summary>
            HybridMeasurement1 = 2,
            /// <summary>
            /// use SL if the max concavity is larger than tau
            /// if smaller than tau, if NeedSP is checked to see of SP is needed 
            /// </summary>
            HybridMeasurement2 = 3
        }

        /// <summary>
        /// Decomposes a concave polygon into smaller, "approximately" convex subpolygons.
        /// Code in https://github.com/jmlien/acd2d has been ported to native C# here (with a major rework and bug fixes).
        /// Larger parts of the algorithm are changed, BridgeAllHolesToOuter is a completely new idea. 
        /// Paper: https://www.sciencedirect.com/science/article/pii/S0925772105001008
        /// Only ConcavityMeasure.StraightLineMeasurement is implemented.
        /// </summary>
        /// <param name="genPoly">polygon to decompose</param>
        /// <param name="tolerance">allowed tolerance for concavity measurement. If zero, resulting polygons are convex</param>
        /// <returns></returns>
        public static List<Polygon2d> DecomposePolygon(this g3.GeneralPolygon2d genPoly, double tolerance) => genPoly.DecomposePolygon(tolerance, out _);

        public static List<Polygon2d> DecomposePolygon(this g3.GeneralPolygon2d genPoly, double tolerance, out List<Segment2d> cuts)
        {
            //sanitize input with clipper, this eliminates any self intersections etc.
            var result = new List<Polygon2d>();            
            var polys = Clipper2Wrapper.Union([genPoly]);
            tolerance = Math.Max(tolerance, 0);
            cuts = new List<Segment2d>();

            foreach (GeneralPolygon2d cleanPoly in polys)
            {
                //use Peucker with shifted Vertex[0] to eliminate collinear edges and duplicate vertices
                cleanPoly.SimplifyShifted();

                if (cleanPoly.Holes.Count == 0 && cleanPoly.Outer.IsConvex())
                {
                    result.Add(cleanPoly.Outer);
                }
                else
                {
                    foreach (var poly in cleanPoly.OuterAndHolesItr())
                    {
                        poly.RemoveCoincidentColinear(Constants.DEFAULT_LINE_IDENTITY_TOLERANCE, Constants.DEFAULT_LINE_IDENTITY_TOLERANCE);
                    }
                    result.AddRange(DecomposePolygonInternal(cleanPoly, tolerance, cuts));
                }
            }

            Debug.Assert(!result.Any(x => x == null));
            return result;
        }

        private static List<Polygon2d> DecomposePolygonInternal(this g3.GeneralPolygon2d genPoly, double tolerance, List<Segment2d> cuts)
        {
            var toDoStack = new Stack<BridgeResult>();
            Polygon2d outer;
            if (genPoly.Holes.Count > 0)
                outer = BridgeAllHolesToOuter(genPoly);
            else
                outer = genPoly.Outer;

            toDoStack.Push(GetBridges(outer));

            var result = new List<Polygon2d>();

            while (toDoStack.Count > 0)
            {
                BridgeResult bridgeResult = toDoStack.Pop();
                double maxConcavity = bridgeResult.IsConvex ? 0.0 : bridgeResult.Bridges.Max(x => x.maxConcavity);
                if (maxConcavity <= tolerance)
                    result.Add(bridgeResult.Polygon);
                else
                {
                    var cutResult = CutOnce(bridgeResult);
                    toDoStack.Push(cutResult.poly1);
                    toDoStack.Push(cutResult.poly2);
                    cuts.Add(cutResult.cutSegment);
                }
            }

            return result;
        }

        private static (BridgeResult poly1, BridgeResult poly2, Segment2d cutSegment) CutOnce(BridgeResult bridges)
        {
            Vector2i cut = FindAGoodCutline(bridges);
            (var poly1, var poly2) = bridges.Polygon.Cut(cut, Constants.DEFAULT_LINE_IDENTITY_TOLERANCE);
            var bResult1 = GetBridges(poly1);
            var bResult2 = GetBridges(poly2);
            var vtcs = bridges.Polygon.Vertices;
            return (bResult1, bResult2, new Segment2d(vtcs[cut.x], vtcs[cut.y]));
        }

        public static Polygon2d BridgeAllHolesToOuter(GeneralPolygon2d genPoly)
        {
            var nHoles = genPoly.Holes.Count;
            var candidates = new List<HoleResolveNode>(nHoles);
            var dependencyGraph = new HoleResolveNode(genPoly, genPoly.Outer) { holeIdx = -1, resolvingHoleIdx = -1, };

            //generate initial candidates for connecting to outer
            for (int i = 0; i < nHoles; i++)
            {
                var hole = genPoly.Holes[i];
                var candidate = dependencyGraph.ClosestConvexCutLine(i, checkOtherHoleCollisions: false);
                if (candidate == null)
                {
                    candidate = dependencyGraph.NearestNeighborVertex(i);
                }
                if (candidate == null)
                {
                    throw new ApplicationException("convex decomposition: failed to find non intersecting nearest neighbor");
                }
                candidates.Add(candidate);
            }
            candidates.SortBy(x => x.bridgeLength);

            var toFallBack = new List<int>();

            //build the graph recursively from root with nearest neighbor
            while (dependencyGraph.ChildCount < nHoles && candidates.Count > 0)
            {
                var connection = candidates[0];
                var iHole = connection.holeIdx;

                //update the best connection with occupied vertices, hole intersection test, etc.
                connection = dependencyGraph.ClosestConvexCutLine(iHole);

                AxisAlignedBox2d? connectionAabb = connection?.polygon.Bounds;
                //try finding a shorter bridge
                foreach (var child in dependencyGraph.Iterator.Skip(1))
                {
                    if (connectionAabb.HasValue)
                    {
                        //perf: do bounds check first
                        var childAabb = child.polygon.Bounds;
                        var aabbDist = connectionAabb.Value.Distance(childAabb);
                        if (aabbDist > connection.bridgeLength)
                            continue;
                    }
                    var candidate = child.ClosestConvexCutLine(iHole);
                    if (candidate == null) continue;

                    if (connection == null || candidate.bridgeLength < connection.bridgeLength)
                    {
                        //most of the time this check is not necessary since we search the shortest bridge
                        //but due to the fallback and the occupied indices I'm not sure it would always work out
                        //if (IntersectsAnyHoles(genPoly, candidate.BridgeSegment, iHole, child.holeIdx).Count > 0)
                        //    continue;
                        connection = candidate;
                    }
                }

                if (connection == null)
                {
                    if (!toFallBack.Contains(iHole))
                    {
                        //remember this hole, remove it from the start of the candidate list and append it at the end
                        //to try to handle it again later
                        toFallBack.Add(iHole);
                        connection = candidates[0];
                        candidates.RemoveAt(0);
                        candidates.Add(connection);
                        continue;
                    }
                    else
                    {
                        //visiting again, resolve by nearest neighbor or inserting a new vertex
                        connection = dependencyGraph.NearestNeighborVertex(iHole);
                        if (connection == null)
                            throw new ApplicationException("convex decomposition: failed to find non intersecting nearest neighbor");
                    }
                }
                //add the new connection into the graph
                connection.parent.InsertChildNode(connection);
                candidates.RemoveAll(x => iHole == x.holeIdx);
            }

            if (dependencyGraph.ChildCount < nHoles)
            {
                throw new ApplicationException("convex decomposition: failed to bridge all holes to outer polygon");
            }

            return dependencyGraph.GetBridgedPolygon();
        }

        private static bool IntersectsAnyHoles(GeneralPolygon2d genPoly, Segment2d cutLine, int holeToIgnore, int hole2ToIgnore = -2)
        {
            for (int i = -1; i < genPoly.Holes.Count; i++)
            {
                if (i == holeToIgnore || i == hole2ToIgnore) continue;
                g3.Polygon2d hole = i == -1 ? genPoly.Outer : genPoly.Holes[i];
                var bounds = hole.Bounds;

                if ((bounds.Contains(cutLine) || cutLine.Intersects(bounds, MathUtil.ZeroTolerancef)) &&
                    hole.IntersectsIgnoreContainment(cutLine, MathUtil.ZeroTolerancef, MathUtil.ZeroTolerancef))
                {
                    return true;
                }
            }

            return false;
        }

        internal sealed class ConcavityMeasureBridge : IConvexHullBridge
        {
            public Vector2i bridgeIndices;
            public double maxConcavity;
            public int notchIdx;
            public List<ConcavityValue> concavityValues;

            public int StartIdx => bridgeIndices.x;

            public int EndIdx => bridgeIndices.y;
        }

        private static BridgeResult GetBridges(Polygon2d polygon) => ConvexHullBridgeFinder.GetBridges(polygon, FindMaxNotch);

        internal struct ConcavityValue
        {
            public int idx;
            public double concavity;

            public ConcavityValue(int idx, double concavity)
            {
                this.idx = idx;
                this.concavity = concavity;
            }
        }

        /// <summary>
        /// Implements the "straight line" concavity measure
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="bridgeIndices"></param>
        /// <returns></returns>
        private static ConcavityMeasureBridge FindMaxNotch(Polygon2d polygon, Vector2i bridgeIndices, IReadOnlyList<Vector2d> maybeMovedVertices)
        {
            var bridge = new ConcavityMeasureBridge()
            {
                bridgeIndices = bridgeIndices,
            };

            //implicit, distance is non-negative: bridge.maxConcavity = 0;

            //use the shifted vertices from the hull construction, to get the (more) correct bridge line
            var vtx1 = maybeMovedVertices[bridgeIndices.x];
            var vtx2 = maybeMovedVertices[bridgeIndices.y];
            Line2d bridgeLine = Line2d.FromPoints(vtx1, vtx2);
            int start;
            int count;
            if (bridgeIndices.y > bridgeIndices.x)
            {
                start = bridgeIndices.x + 1;
                count = bridgeIndices.y - bridgeIndices.x - 1;
            }
            else
            {
                start = (bridgeIndices.x + 1) % polygon.VertexCount;
                count = polygon.VertexCount - bridgeIndices.x + bridgeIndices.y - 1;
            }
            bridge.concavityValues = new List<ConcavityValue>(count / 2);

            double maxDistSeen = double.MinValue;
            int i = start;
            for (int j = 0; j < count; j++)
            {
                int iWrapped = i % polygon.VertexCount;
                Vector2d vtx = polygon[iWrapped];
                var distance = bridgeLine.DistanceSquared(vtx);
                maxDistSeen = Math.Max(maxDistSeen, distance);
                bridge.concavityValues.Add(new ConcavityValue(iWrapped, distance));
                if (distance >= bridge.maxConcavity)
                {
                    if (distance > bridge.maxConcavity || !polygon.IsVertexConvex(iWrapped))
                    {
                        bridge.maxConcavity = distance;
                        bridge.notchIdx = iWrapped;
                    }
                }
                i++;
            }
            //assert Lemma 6.2 (see Paper Approximate convex decomposition of polygons)
            //it might be that the removed vertices are almost colinear
            //(we have to use a tolerance for the convex hull to handle "touching" vertices on edges)
            //the deepest notch MUST be concave, if it is not the polygon is not simple => has self intersection
            //since we use clipper to clean the input, we must have created a self intersection internally
            if (polygon.IsVertexConvex(bridge.notchIdx))
            {
                //check if the vertices are nearly parallel to the bridge
                if (maxDistSeen < ConvexHull2.DefaultConvexHullTolerance)
                    return null;
                else
                    throw new ApplicationException($"self intersection created during decomposition. maxDistSeen {maxDistSeen} notchIdx {bridge.notchIdx}");
            }
            //do Sqrt once only
            bridge.maxConcavity = Math.Sqrt(bridge.maxConcavity);
            return bridge;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="poly"></param>
        /// <returns></returns>
        private static Vector2i FindAGoodCutline(BridgeResult bridgeResult)
        {
            //find max scoring vertex, score is the concavity measure of the vertex
            //to check if a given candidate resolves the notch we calculate the "visibility score"
            //that tells as if the cut line target is in the inner cone of the notch
            //the polygons resulting from the cut will only be convex if the target is in the inner cone
            var bridges = bridgeResult.Bridges;
            var polygon = bridgeResult.Polygon;

            var allConcavatyValues = bridges.Select(x => x.concavityValues).Flatten();

            allConcavatyValues.SortByDescending(x => x.concavity);

            Vector2i cutLine = new Vector2i();
            bool isResolved = false;

            int notchIdx = allConcavatyValues[0].idx;
            cutLine.x = notchIdx;
            List<(int visibilityScore, ConcavityValue concavity)> fallback = new();
            //first check if bridge candidates resolve the max notch
            for (int i = 1; i < allConcavatyValues.Length; i++)
            {
                var concavity = allConcavatyValues[i];
                //skip vertices adjacent to bridge support
                if (concavity.idx >= cutLine.x - 1 &&
                    concavity.idx <= cutLine.x + 1)
                    continue;
                cutLine.y = concavity.idx;

                //check for coincidence and resolve it as soon as it shows up
                if (polygon.AreVerticesCoincident(cutLine.x, cutLine.y))
                {
                    // different bridges?
                    int notchBridgeIdx = bridgeResult.GetBridgeIndex(cutLine.x);
                    if (notchBridgeIdx == -1 || notchBridgeIdx != bridgeResult.GetBridgeIndex(cutLine.y))
                        return cutLine;
                    else
                    {   //rare edge case, there is a coincident vertex for the notch
                        //use the vertex whose bisector normal points away from the bridge
                        var normal = polygon.GetNormal_FaceAvg(cutLine.x);
                        var bridgeIdcs = bridgeResult.Bridges[notchBridgeIdx].bridgeIndices;
                        var bridgeDirection = new Vector2d(bridgeIdcs[0], bridgeIdcs[1]);
                        if (normal.Dot(bridgeDirection) < 0)
                        {
                            //change notch and restart loop
                            cutLine.x = concavity.idx;
                            notchIdx = concavity.idx;
                        }
                        continue;
                    }
                }

                // give a higher tolerance for "on line" visibility score, which will result in near collinear segments
                // if the segments are colinear enough, they will be removed after the cut
                int visibilityScore = VisibilityScore(polygon, cutLine, 1e-3);

                //check if the vertex is in the inner visibility cone
                //if it is not, the new polygons vertices at the cut will not be convex
                if (visibilityScore < 0)
                    continue;

                if (visibilityScore == 10 && !polygon.IsVertexConvex(concavity.idx))
                {
                    //the perfect cutLine, visibility in inner cone on both sides, resolves two notches
                    isResolved = !IsIntersecting(polygon, cutLine);
                    if (isResolved)
                        return cutLine;
                }
                else
                {
                    fallback.Add((visibilityScore, concavity));
                }
            }

            const int visibilityThreshold = 4;
            //fallback 1: no perfect cut line here, lower our expectations
            //accept if target notch is not resolved or creates collinear segments (within visibility tolerance)
            var resolvingNotches = fallback
                .Where(x => x.visibilityScore > visibilityThreshold)
                .OrderByDescending(x => x.concavity.concavity)
                .ThenByDescending(x => x.visibilityScore);
            foreach (var candidate in resolvingNotches)
            {
                cutLine.y = candidate.concavity.idx;
                isResolved = !IsIntersecting(polygon, cutLine);
                if (isResolved)
                    return cutLine;
            }

            //fallback 2: there is no concavity vertex that resolves the notch
            //iterate convex vertices (they all have 0 concavity score)
            double maxScore = -1;
            int best = -1;
            var convexVertices = bridgeResult.ConvexHull.HullIndices;
            for (int i = 0; i < convexVertices.Length; i++)
            {
                var idx = convexVertices[i];
                //skip vertices adjacent to bridge support
                if (idx >= cutLine.x - 1 &&
                    idx <= cutLine.x + 1)
                    continue;
                cutLine.y = idx;
                if (polygon.AreVerticesCoincident(cutLine.x, cutLine.y))
                    return cutLine;

                int visibilityScore = VisibilityScore(polygon, cutLine);
                //only ever use convex vertices that resolve the notch
                if (visibilityScore < 8)
                    continue;
                var cutLength = (polygon[cutLine.x] - polygon[cutLine.y]).LengthSquared;
                var score = visibilityScore / cutLength;
                if (score > maxScore && !IsIntersecting(polygon, cutLine))
                {
                    maxScore = score;
                    best = idx;
                }
            }

            if (best > -1)
            {
                cutLine.y = best;
                return cutLine;
            }

            //fallback 3: there is no good vertex that resolves the notch. Check if any vertices from fallback 1 belong to another bridge
            //these fallback vertices have visiblity score below 8, thus the do not fully resolve the max notch
            //but if we resolve them we might change the maxNotch location without adding a new vertex (the last resort, fallback 4)
            //to prevent unnecessary notch resolves we also filter out vertices that have concavity below tolerance and might not need to be resolved later
            var notchBridge = bridgeResult.GetBridgeIndex(notchIdx);
            var bridgedCandidates = fallback.Where(x => x.visibilityScore <= visibilityThreshold && bridgeResult.GetBridgeIndex(x.concavity.idx) != notchBridge);
            foreach (var bridgedCandidate in bridgedCandidates.OrderBy(x => (polygon[x.concavity.idx] - polygon[cutLine.x]).LengthSquared))
            {
                cutLine.y = bridgedCandidate.concavity.idx;
                //coincidence of target neighbors might slip through here, because visibility score might have been 0
                if (bridgedCandidate.visibilityScore < 2)
                {
                    if (polygon.AreVerticesCoincident(cutLine.x + 1, cutLine.y))
                    {
                        cutLine.x++;
                        return cutLine;
                    }
                    if (polygon.AreVerticesCoincident(cutLine.x - 1, cutLine.y))
                    {
                        cutLine.x--;
                        return cutLine;
                    }
                }
                isResolved = !IsIntersecting(polygon, cutLine);
                if (isResolved)
                    return cutLine;
            }

            //fallback 4: there are no vertices that resolve the notch, add a new vertex at the angle bisector
            cutLine.x = notchIdx;
            var bisector = GetBisectorIntersection(polygon, cutLine.x);
            if (bisector.index < 0)
                throw new InvalidOperationException("invalid polygon data for decomposition");
            polygon.InsertVertex(bisector.index, bisector.intersection);
            cutLine.y = bisector.index;
            if (cutLine.x > cutLine.y) cutLine.x += 1;

            return cutLine;
        }

        private static int VisibilityScore(Polygon2d polygon, Vector2i cutLine, double tolerance = 0)
        {
            //1. check direction of cutline, if the direction is not away from notch its out
            var whichSideSupport = polygon.WhichSideAt(cutLine.x, polygon[cutLine.y], tolerance);
            if (whichSideSupport < 0)
                return whichSideSupport;
            //2. check direction at bridge target as well
            var whichSideTarget = polygon.WhichSideAt(cutLine.y, polygon[cutLine.x], tolerance);
            if (whichSideTarget < 0)
                return whichSideTarget;
            else
                return whichSideSupport * 4 + whichSideTarget;
        }

        const double tolLine = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE;
        private static bool IsIntersecting(g3.Polygon2d polygon, int idx, Vector2d cutLineTarget)
        {
            var cutLineSeg = new Segment2d(polygon[idx], cutLineTarget);
            //check for coincident points, cutting at those is fine
            if (cutLineSeg.Length < tolLine)
                return false;
            return IsIntersecting(polygon, cutLineSeg, idx);
        }

        private static bool IsIntersecting(g3.Polygon2d polygon, Vector2i cutLine)
        {
            var cutLineSeg = new Segment2d(polygon[cutLine.x], polygon[cutLine.y]);
            //check for coincident points, cutting at those is fine
            if (cutLineSeg.Length < tolLine)
                return false;
            return IsIntersecting(polygon, cutLineSeg, cutLine.x, cutLine.y);
        }

        private static bool IsIntersecting(g3.Polygon2d polygon, Segment2d cutLineSeg, int idxToIgnore, int idxToIgnore2 = -1)
        {
            int prev = polygon.VertexCount - 1;
            for (int i = 0; i < polygon.VertexCount; i++)
            {
                var segment = new Segment2d(polygon[prev], polygon[i]);
                if (segment.Intersects(cutLineSeg, tolLine, tolLine)
                    && i != idxToIgnore
                    && prev != idxToIgnore
                    && i != idxToIgnore2
                    && prev != idxToIgnore2)
                    return true;
                prev = i;
            }

            return false;
        }

        private static (Vector2d intersection, int index) GetBisectorIntersection(Polygon2d polygon, int idx)
        {
            var bisectorLine = new Line2d(polygon[idx], polygon.GetNormal_FaceAvg(idx));

            int bestI = -1;
            Vector2d bestIntersection = Vector2d.MaxValue;
            double minDist = double.MaxValue;
            int prev = polygon.VertexCount - 1;
            for (int i = 0; i < polygon.VertexCount; prev = i++)
            {
                if (i == idx || prev == idx)
                    continue;

                var segment = new Segment2d(polygon[prev], polygon[i]);
                var intersectionPoint = bisectorLine.IntersectionPoint(segment);
                if (intersectionPoint == Vector2d.MaxValue)
                    continue;

                var dist = (bisectorLine.Origin - intersectionPoint).LengthSquared;
                if (dist > minDist)
                    continue;

                if (dist < Constants.DEFAULT_LINE_IDENTITY_TOLERANCE)
                {
                    //the intersection point is on the segment, immediately return it
                    return (intersectionPoint, i);
                }

                var visibility = polygon.WhichSideAt(idx, intersectionPoint);
                if (visibility < 0)
                    continue;

                if (dist == minDist || visibility < 2)
                {
                    //dist == minDist => two segments with an intersetion are coincident => one of our cuts from hole resolve step
                    //visibility < 2 => bisector line and notch edges are approx. colinear => the notch is a approx. 180° sharp spike
                    //decide using the normal direction in both cases
                    if (segment.Direction.Perp.Dot(bisectorLine.Direction) > 0)
                        continue;
                }

                minDist = dist;
                bestI = i;
                bestIntersection = intersectionPoint;
            }

            return (bestIntersection, bestI);
        }
    }
}
