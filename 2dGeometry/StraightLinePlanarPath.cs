using g3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.Sockets;
using BridgeResult = g3.ConvexHullBridgeFinder.BridgeResult<g3.ConvexHullBridgeFinder.Bridge>;
using BridgeResultMultiPoly = g3.ConvexHullBridgeFinder.BridgeResult<g3.ConvexHullBridgeFinder.MultiPolyBridge>;

namespace g3
{
    public class StraightLinePlanarPath
    {
        /// <summary>
        /// Escape path result.
        /// If start is contained inside a polygon hole, a planar escape path does not exist.
        /// If start is inside a concavity, all paths begin by escaping the concavity 
        /// and then follow the convex hulls of the obstacles.
        /// If there are multiple obstacles, multiple escape paths are generated, one each per bridge of the convex hull.
        /// </summary>
        public class ConvexHullEscape
        {
            public EscapeResult Result { get; }
            public PolyLine2d[] Paths { get => paths; init => paths = value; }
            /// <summary>
            /// The obstacle concavity or hole containing the start point.
            /// </summary>
            public Polygon2d ContainingCavity => Result switch
            {
                EscapeResult.OutsideConvexHull
                    => throw new InvalidOperationException($"Result is {Result}, containing concavity does not exist"),
                _ => _containingCavity,
            };
            public int CavityIdx => Result switch
            {
                EscapeResult.ContainedInHole => _cavityIdx,
                _ => throw new InvalidOperationException($"Result is {Result}, CavityIdx does not exist"),
            };
            private Polygon2d _containingCavity;
            private int _cavityIdx;
            private PolyLine2d[] paths;

            private ConvexHullEscape(EscapeResult result, PolyLine2d[] paths = null, Polygon2d containingCavity = null, int cavityIdx = 0)
            {
                Result = result;
                Paths = paths;
                _containingCavity = containingCavity;
                _cavityIdx = cavityIdx;
            }

            public static class ConvexHullEscapeHelper
            {
                public static ref PolyLine2d[] PathsRef(ConvexHullEscape escape) => ref escape.paths;
            }

            internal static ConvexHullEscape ContainedInHole(Polygon2d hole, int holeIdx) => new ConvexHullEscape(EscapeResult.ContainedInHole, Array.Empty<PolyLine2d>(), hole, holeIdx);
            internal static ConvexHullEscape ContainedInConcavity(PolyLine2d path, Polygon2d cavity) => new ConvexHullEscape(EscapeResult.ContainedInConcavity, [path], cavity);
            internal static ConvexHullEscape BetweenConvexObstacles(PolyLine2d[] paths) => new ConvexHullEscape(EscapeResult.BetweenConvexObstacles, paths, null);
            internal static ConvexHullEscape InConcavityBetweenConvexObstacles(PolyLine2d[] paths, Polygon2d cavity) => new ConvexHullEscape(EscapeResult.InConcavityBetweenConvexObstacles, paths, cavity);
            internal static ConvexHullEscape InsideObstacle(Polygon2d obstacle) => new ConvexHullEscape(EscapeResult.InsideObstacle, Array.Empty<PolyLine2d>(), obstacle);

            internal static readonly ConvexHullEscape OutsideConvexHull = new ConvexHullEscape(EscapeResult.OutsideConvexHull, Array.Empty<PolyLine2d>());
        }

        public enum EscapeResult
        {
            OutsideConvexHull,
            ContainedInHole,
            ContainedInConcavity,
            BetweenConvexObstacles,
            InConcavityBetweenConvexObstacles,
            InsideObstacle,
        }

        /// <summary>
        /// Computes approximately short paths from start to different exits of the convex hull of the obstacles constellation.
        /// Inspired by Bug2 path planning algorithm.
        /// If the direct path is obstructed, follows the shorter way around the obstacle boundary until the straight path is encountered again.
        /// Unlike visibility graph based paths, the algorithm has linear time complexity to the number of obstacle edges.
        /// Will exit a cavity if start is located inside one, and then follow the convex hulls of the obstacles.
        /// </summary>
        /// <param name="obstacles">List of polygonal obstacles</param>
        /// <param name="start">Start point</param>
        /// <param name="tol">collision tolerance</param>
        /// <returns>List of waypoints representing the path.</returns>
        public static ConvexHullEscape ConvexHullEscapePath(Polygon2d[] obstacles, Vector2d start, double tol = MathUtil.ZeroTolerancef)
        {
            //this check is considerably more expensive than the whole method => DEBUG
            Debug.Assert(obstacles.HaveIntersections(allowPointContact: true) == false);

            //start with point in polygon checks in holes, since computing convex hulls is more expensive than PiP
            int holeIdx = obstacles.IndexOfContainingHole(start, tol);
            //if start is located inside a hole, it is impossible to reach the convex hull of the obstacle assembly
            if (holeIdx > -1)
            {
                return ConvexHullEscape.ContainedInHole(obstacles[holeIdx], holeIdx);
            }

            List<BridgeResult> bridgeResults = new();
            foreach (Polygon2d outer in obstacles.Where(o => !o.IsHole))
            {
                var bridgeRes = ConvexHullBridgeFinder.ConvexHullBridges(outer);
                bridgeResults.Add(bridgeRes);
            }

            // nothing or holes only and not contained?
            if (bridgeResults.Count == 0)
            { return ConvexHullEscape.OutsideConvexHull; }

            // single general polygon?
            if (bridgeResults.Count == 1)
            {
                return SinglePolyEscape(bridgeResults[0], start, tol, out _, out _);
            }

            Polygon2d[] hullObstacles = new Polygon2d[bridgeResults.Count];
            HashSet<int> concavityBridgeIdcs = new();
            int offsetIdx = 0, writeIdx = 0;
            bool hasConcave = false;
            Polygon2d containingCavity = null;
            // there are multiple obstacles, we need multiple escape paths
            foreach (BridgeResult bridgeRes in bridgeResults)
            {
                if (bridgeRes.IsConvex)
                {
                    if (bridgeRes.Polygon.ContainsExclusive(start, tol)) { return ConvexHullEscape.InsideObstacle(bridgeRes.Polygon); }
                    hullObstacles[writeIdx++] = bridgeRes.Polygon;
                    offsetIdx += bridgeRes.Polygon.VertexCount;
                    continue;
                }
                ConvexHullEscape escape = SinglePolyEscape(bridgeRes, start, tol, out Polygon2d hullPoly, out int bridgeIdx, skipPathCompute: true);
                if (escape.Result == EscapeResult.ContainedInConcavity)
                {
                    hasConcave |= bridgeRes.Bridges.Count > 1;
                    Polygon2d hullWithPocket = bridgeRes.ConvexHullWithPocketForBridge(bridgeIdx);
                    var bridge = bridgeRes.Bridges[bridgeIdx];
                    int localOffset = bridgeRes.Bridges.Take(bridgeIdx).Sum(x => x.BridgedVtxCount(bridgeRes.Polygon.VertexCount) - 2);
                    concavityBridgeIdcs.Add(offsetIdx + bridge.StartIdx - localOffset);
                    concavityBridgeIdcs.Add(offsetIdx + bridge.EndIdx - localOffset);
                    hullObstacles[writeIdx++] = hullWithPocket;
                    offsetIdx += hullWithPocket.VertexCount;
                    if (containingCavity != null)
                    {
                        containingCavity = UnionCavities(obstacles, start, tol, containingCavity, escape.ContainingCavity);
                    }
                    else
                    {
                        containingCavity = escape.ContainingCavity;
                    }
                }
                else if(escape.Result == EscapeResult.InsideObstacle)
                {
                    return escape;
                }
                else
                {
                    Debug.Assert(escape.Result == EscapeResult.OutsideConvexHull);
                    hasConcave = true;
                    hullObstacles[writeIdx++] = hullPoly;
                    offsetIdx += hullPoly.VertexCount;
                }
            }

            if(hasConcave) hullObstacles = hullObstacles.Union();
            BridgeResultMultiPoly assemblyHull = ConvexHullBridgeFinder.GetBridges(hullObstacles);
            if (!assemblyHull.HullPolygon.ContainsExclusive(start, tol)) { return ConvexHullEscape.OutsideConvexHull; }

            IReadOnlyList<ConvexHullBridgeFinder.MultiPolyBridge> bridges = assemblyHull.Bridges;
            if (bridges.Count < 1) throw new InvalidOperationException("assembly convex hull must have at least 1 bridge");

            if (bridges.Count > 1)
            {
                var pockets = ConvexHullBridgeFinder.BridgedPocket.GetPockets(hullObstacles, bridges);
                int counter = 0;
                foreach (var pocket in pockets)
                {
                    if (pocket.Polygon.ContainsInclusive(start, tol))
                    {
                        bridges = pocket.Bridges;
                        counter++;
                    }
                }
                Debug.Assert(counter == 1);
            }

            HashSet<Vector2d> assemblyHullCorners = MakeBridgeCornerSet(assemblyHull);

            PolyLine2d[] escapePaths = new PolyLine2d[bridges.Count];
            for (int i = 0; i < bridges.Count; i++)
            {
                Segment2d bridgeSegment = bridges[i].BridgeSegment(assemblyHull.Vertices);
                Vector2d goal = bridgeSegment.NearestPoint(start);
                PolyLine2d path = ComputePath(hullObstacles, start, goal, tol, treatObstaclesAsPolylines: false);
                TruncatePathIfMeetsHull(assemblyHullCorners, path);
                escapePaths[i] = path;
            }

            ConvexHullEscape result;
            if (containingCavity == null)
            {
                result = ConvexHullEscape.BetweenConvexObstacles(escapePaths);
            }
            else
            {
                result = ConvexHullEscape.InConcavityBetweenConvexObstacles(escapePaths, containingCavity);
            }
            return result;
        }

        private static Polygon2d UnionCavities(Polygon2d[] obstacles, Vector2d start, double tol, Polygon2d existingCavity, Polygon2d newCavity)
        {
            Polygon2d[] union = existingCavity.Union(newCavity);
            // --- happy path, this is what we expect 99% of time ---
            if (union.Length == 1)
                return union[0];

            if (union.Length == 0)
                throw new ArgumentException("input polygons empty");

            // --- handle multiple polygons in union ---
            //case 1: if a polygon "touches" itself, clipper might have split it into 2 at the self touch.
            //Check again if all polys contain start
            int contained = 0;
            Polygon2d cavityUnion = null;
            foreach (var poly in union)
            {
                if (poly.ContainsInclusive(start, tol))
                {
                    cavityUnion = poly;
                    contained++;
                }
            }
            if (contained == 1)
                return cavityUnion;

            //case 2: because we have a tolerance for the containment checks
            //it is possible that start is "contained" in 2 concavities very close to each other, but not connected
            //try to connect both concavities with a tiny square around start
            Polygon2d tinySquare = Polygon2d.MakeRectangle(start, tol, tol);
            if (newCavity.IsHole) tinySquare.Reverse();
            union = union.Union(tinySquare);
            if (union.Length > 1)
            {
                //how is this possible???
                SVGWriter writer = new();
                writer.AddPolygons(obstacles, SVGWriter.Style.Outline("black", 1));
                writer.AddPoint(start, SVGWriter.Style.Outline("green", 1));
                writer.AddPolygons(union, SVGWriter.Style.Outline("red", 1));
                string file = writer.WriteDebug();
                throw new InvalidOperationException($"concavities should intersect, debug export {file}");
            }

            return union[0];
        }

        private static void TruncatePathIfMeetsHull(HashSet<Vector2d> assemblyHullCorners, PolyLine2d path)
        {
            for (int i = 0; i < path.Vertices.Count; i++)
            {
                Vector2d vtx = path.Vertices[i];
                if (assemblyHullCorners.Contains(vtx))
                {
                    path.TrimVerticesTo(i + 1);
                    return;
                }
            }
        }

        private static HashSet<Vector2d> MakeBridgeCornerSet(BridgeResultMultiPoly assemblyHull)
        {
            HashSet<Vector2d> assemblyHullCorners = new();
            foreach (ConvexHullBridgeFinder.MultiPolyBridge bridge in assemblyHull.Bridges)
            {
                assemblyHullCorners.Add(assemblyHull.Vertices[bridge.StartIdx]);
                assemblyHullCorners.Add(assemblyHull.Vertices[bridge.EndIdx]);
            }

            return assemblyHullCorners;
        }

        /// <summary>
        /// Computes approximately short paths from start to different exits of the convex hull of the obstacle.
        /// Inspired by Bug2 path planning algorithm.
        /// If the direct path is obstructed, follows the shorter way around the obstacle boundary until the straight path is encountered again.
        /// Unlike visibility graph based paths, the algorithm has linear time complexity to the number of obstacle edges.
        /// </summary>
        /// <param name="obstacle">a polygonal obstacle</param>
        /// <param name="start">start point</param>
        /// <param name="tol">collision tolerance</param>
        /// <returns>List of waypoints representing the path.</returns>
        public static ConvexHullEscape ConvexHullEscapePath(Polygon2d obstacle, Vector2d start, double tol = MathUtil.ZeroTolerance)
        {
            var bridgeRes = ConvexHullBridgeFinder.ConvexHullBridges(obstacle);
            return SinglePolyEscape(bridgeRes, start, tol, out _, out _);
        }

        private static ConvexHullEscape SinglePolyEscape(BridgeResult bridgeRes, Vector2d start, double tol, out Polygon2d hullPoly, out int bridgeIdx, bool skipPathCompute = false)
        {
            Polygon2d outer = bridgeRes.Polygon;
            hullPoly = bridgeRes.HullPolygon;

            bridgeIdx = -1;
            bool isContained = hullPoly.ContainsExclusive(start, tol);
            if (!isContained) { return ConvexHullEscape.OutsideConvexHull; }
            // if the input is convex, we are already done:
            // if start is not contained, it must be inside the single convex obstacle
            if (bridgeRes.IsConvex) { return ConvexHullEscape.InsideObstacle(outer); }

            foreach (Polygon2d cavity in bridgeRes.BridgedConcavitiesAsPolygon())
            {
                bridgeIdx++;
                if (!cavity.ContainsInclusive(start, tol)) continue;
                Segment2d bridge = bridgeRes.Bridges[bridgeIdx].BridgeSegment(bridgeRes.Vertices);
                Vector2d goal = bridge.NearestPoint(start);
                if (bridge.DistanceSquared(start) < tol * tol) { return ConvexHullEscape.OutsideConvexHull; }
                PolyLine2d path = null;
                if(!skipPathCompute) path = ComputePath([cavity], start, goal, treatObstaclesAsPolylines: true);
                return ConvexHullEscape.ContainedInConcavity(path, cavity);
            }
            bridgeIdx = -1;

            //contains method is not always numerically stable.
            //so in the rare error case, we compute the distance, where we don't know if the point lies that far inside or outside
            //contains should be positive for cases where the distance is too far to the outside
            //so we only check if the point is within tolerance of the convex hull, in which case we consider it outside
            double distanceSqr = hullPoly.DistanceSquared(start);
            if (distanceSqr < tol * tol) { return ConvexHullEscape.OutsideConvexHull; }
            //start is inside poly
            return ConvexHullEscape.InsideObstacle(outer);
        }

        public static PolyLine2d ComputePath(IReadOnlyList<Polygon2d> obstacles, Vector2d start, Vector2d goal, double tol = MathUtil.ZeroTolerance, bool treatObstaclesAsPolylines = false)
        {
            // m–line from start to goal.
            Segment2d mLine = new Segment2d(start, goal);
            double tolSqd = tol * tol;
            List<Detour> detours = FindDetours(obstacles, tol, treatObstaclesAsPolylines, mLine);

            if (detours.Count == 0) return mLine.ToPolyLine();

            Debug.Assert(treatObstaclesAsPolylines || Math.Abs(detours.Sum(dt => dt.dist) - detours.Select(dt => dt.ObstacleIndex).Distinct().Select(i => obstacles[i]).Sum(obs => obs.ArcLength)) < 1e-3);

            //reverse direction if detour leads backwards on m-line
            foreach (Detour detour in detours)
            {
                if (detour.tStart <= detour.tEnd) continue;
                int wrapIdx = obstacles[detour.ObstacleIndex].VertexCount - 1;
                int temp = detour.startIdx;
                detour.startIdx = detour.endIdx == 0 ? wrapIdx : detour.endIdx - 1;
                detour.endIdx = temp == 0 ? wrapIdx : temp - 1;
                double temp2 = detour.tStart;
                detour.tStart = detour.tEnd;
                detour.tEnd = temp2;
                detour.IsClockwise = false;
            }

            detours.SortBy(d => d.tStart);

            double globalPos = -mLine.Extent - tol;
            List<DetourState> inProcess = [new DetourState() { tCurrent = globalPos, dist = 0.0, idx = 0, startIdx = 0, forkIdx = 0 }];
            List<int> detoursToSkip = new();
            //solve the decision graph using the distance as cost heuristic
            while (globalPos < mLine.Extent)
            {
                for (int i = 0; i < inProcess.Count; i++)
                {
                    DetourState state = inProcess[i];
                    if (state.tCurrent > globalPos) continue;

                    int detourIdx = state.idx;
                    Detour detour = detours[detourIdx];
                    if (detourIdx < detours.Count - 1 && Math.Abs(detours[detourIdx + 1].tStart - detour.tStart) < tol)
                    {
                        //fork, will be processed in later iterations
                        state.forkIdx = state.idx;
                        state.idx++;
                        DetourState fork = state;
                        fork.forkIdx++;
                        inProcess.Add(fork);
                    }
                    //apply the detour, position is now at detour end
                    state.dist += detour.dist;
                    //traverse the mLine from detour end until the next applicable detour is found
                    while (state.idx < detours.Count && detours[state.idx].tStart <= detour.tEnd) { state.idx++; }
                    state.tCurrent = state.idx == detours.Count ? mLine.Extent : detours[state.idx].tStart;
                    state.dist += state.tCurrent - detour.tEnd;
                    inProcess[i] = state;
                }

                //merge?
                for (int i = inProcess.Count - 1; i >= 1; i--)
                {
                    if (inProcess[i].tCurrent != inProcess[i - 1].tCurrent) continue;
                    DetourState state = inProcess[i];
                    DetourState state2 = inProcess[i - 1];
                    if (state.dist > state2.dist)
                    {
                        inProcess.RemoveAt(i);
                        detoursToSkip.Add(state.forkIdx);
                    }
                    else
                    {
                        inProcess.RemoveAt(i - 1);
                        detoursToSkip.Add(state2.forkIdx);
                    }
                }

                globalPos = inProcess.Min(ip => ip.tCurrent);
            }

            // Now build the output path along the m–line while inserting detours.
            double currentT = -2 * tol - mLine.Extent;
            Vector2d currentPoint = start;
            PolyLine2d path = new();
            path.AppendVertex(currentPoint);

            // Process each intersection interval, skipping those that lie within an already processed interval.
            for (int detourIdx = 0; detourIdx < detours.Count; detourIdx++)
            {
                Detour detour = detours[detourIdx];
                if (detour.tStart <= currentT)
                    continue; // This intersection is already covered.
                if (detoursToSkip.Contains(detourIdx))
                    continue;

                // Add clear segment along the m–line from currentPoint to the entry point of this obstacle.
                Vector2d vtxOnMLine = mLine.PointAt(detour.tStart);
                if ((vtxOnMLine - currentPoint).LengthSquared > tolSqd)
                    path.AppendVertex(vtxOnMLine);

                // Add the detour vertices along the obstacle boundary from pEntry to pExit.
                var obstacle = obstacles[detour.ObstacleIndex];
                int i = detour.startIdx;

                int wrapIdx = obstacle.VertexCount - 1;
                Func<int, int> nextIdxWrapped;
                if (detour.IsClockwise)
                    nextIdxWrapped = (i) => i == wrapIdx ? 0 : i + 1;
                else
                    nextIdxWrapped = (i) => i == 0 ? wrapIdx : i - 1;

                while (i != detour.endIdx)
                {
                    Vector2d nextVtx = obstacle[i];
                    if ((nextVtx - currentPoint).LengthSquared > tolSqd)
                    {
                        path.AppendVertex(nextVtx);
                        currentPoint = nextVtx;
                    }
                    i = nextIdxWrapped(i);
                }

                // Update currentT and currentPoint to the exit point.
                currentT = detour.tEnd;
                vtxOnMLine = mLine.PointAt(detour.tEnd);
                if ((vtxOnMLine - currentPoint).LengthSquared > tolSqd)
                    path.AppendVertex(vtxOnMLine);
                currentPoint = vtxOnMLine;
            }

            // Finally, if the last processed point is not the goal, add the remaining clear segment.
            if ((goal - currentPoint).LengthSquared > tolSqd)
                path.AppendVertex(goal);
            else
                path.VerticesAsSpan[^1] = goal;

            //Debug.Assert(path.SegmentItr().All(seg1 => obstacles.SelectMany(x => x.SegmentItr()).All(seg2 => !seg2.IntersectsNotParallel(ref seg1, out _, out _))));

            return path;
        }

        private static List<Detour> FindDetours(IReadOnlyList<Polygon2d> obstacles, double tol, bool treatObstaclesAsPolylines, in Segment2d mLine)
        {
            List<Detour> detours = new();
            // Process each obstacle.
            for (int i = 0; i < obstacles.Count; i++)
            {
                Polygon2d poly = obstacles[i];
                int firstDetourIdx = detours.Count;

                (Detour curDetour, double distStart) = AddDetoursForObstacle(poly, mLine, detours, i, tol * 2, treatObstaclesAsPolylines);
                if (curDetour == null) continue;

                //handle wrap around for last detour, merge with first detour of obstacle
                curDetour.dist += distStart;
                if (curDetour.dist > tol)
                {
                    if (detours.Count > firstDetourIdx)
                    {
                        Detour firstDetour = detours[firstDetourIdx];
                        curDetour.endIdx = firstDetour.startIdx;
                        curDetour.tEnd = firstDetour.tStart;
                        detours.Add(curDetour);
                    }
                    else
                    {
                        // single point of contact, the curDetour would lead around the whole obstacle
                        // drop curDetour
                    }
                }
            }

            return detours;
        }

        private static (Detour curDetour, double distStart) AddDetoursForObstacle(Polygon2d poly, Segment2d mLine, List<Detour> detours, int obstacleIdx, double tol, bool treatObstaclesAsPolylines)
        {
            var verts = poly.VerticesAsReadOnlySpan;
            if (verts.Length < 2) throw new ArgumentException("obstacle polygon has less than 2 vertices");

            // For each edge of the obstacle, test for intersection with mLine.
            Vector2d prev = treatObstaclesAsPolylines ? verts[0] : verts[^1];
            Segment2d edge;
            Detour curDetour = null;
            double distStart = 0;
            double t1 = double.NaN, t2 = double.NaN;
            bool lastWasOnVertex = false;
            int firstVtx = treatObstaclesAsPolylines ? 1 : 0;
            for (int j = firstVtx; j < verts.Length; prev = verts[j++])
            {
                edge = new Segment2d(prev, verts[j]);
                //do not count intersections at vertices twice
                bool intersects = lastWasOnVertex ? false : mLine.IntersectsNotParallel(edge, out t1, out t2, intervalThresh: tol);
                lastWasOnVertex = false;
                if (intersects)
                {
                    bool isOnVertex = Math.Abs(Math.Abs(t2) - edge.Extent) < tol;
                    bool isMLineVertex = Math.Abs(Math.Abs(t1) - mLine.Extent) < tol;
                    if (isMLineVertex && !isOnVertex)
                    {
                        intersects = edge.WhichSide(mLine.Center) == 1;
                    }
                    else if (isOnVertex)
                    {
                        //find the other segment
                        //check special case at wrap around: is this the second segment in clockwise order?
                        //everywhere else we get the intersection with the first segment in clockwise order
                        //note: for the last iteration lastWasOnVertex will have no effect
                        Segment2d otherSeg;
                        if (j == 0 && t2 < 0)
                            otherSeg = poly.Segment(poly.VertexCount - 2);
                        else
                        {
                            otherSeg = poly.Segment(j);
                            lastWasOnVertex = true;
                        }

                        if (isMLineVertex)
                        {
                            //decide if the mLine direction is to the inside or outside of the 2 adjacent edges
                            bool firstIsRight = edge.WhichSide(mLine.Center) == -1;
                            bool secIsRight = otherSeg.WhichSide(mLine.Center) == -1;
                            if (poly.IsVertexConvex(j))
                            {
                                intersects = !(firstIsRight || secIsRight);
                            }
                            else
                            {
                                intersects = !(firstIsRight && secIsRight);
                            }
                        }
                        else
                        {
                            //touching a vertex might or might not be an intersection, depends on if the 2 edges are on different sides of mLine
                            int firstSide = mLine.WhichSide(edge.Center);
                            int secSide = mLine.WhichSide(otherSeg.Center);
                            intersects = firstSide != secSide;
                        }
                    }
                }
                if (intersects)
                {
                    if (curDetour != null)
                    {
                        curDetour.endIdx = j;
                        curDetour.tEnd = t1;
                        curDetour.dist += edge.Extent + t2;
                        detours.Add(curDetour);
                    }
                    else
                    {
                        distStart += edge.Extent + t2;
                    }
                    curDetour = new Detour();
                    curDetour.ObstacleIndex = obstacleIdx;
                    curDetour.tStart = t1;
                    curDetour.dist = edge.Extent - t2;
                    curDetour.startIdx = j;
                }
                else if (curDetour != null)
                {
                    curDetour.dist += edge.Length;
                }
                else
                {
                    distStart += edge.Length;
                }
                prev = verts[j];
            }

            return (curDetour, distStart);
        }

        private class Detour
        {
            public double tStart;
            public double tEnd;
            public double dist;
            public int ObstacleIndex;
            public int startIdx;
            public int endIdx;
            public bool IsClockwise = true;
        }

        private struct DetourState
        {
            public double tCurrent;
            public double dist;
            public int idx;
            public int startIdx;
            public int forkIdx;
        }
    }
}