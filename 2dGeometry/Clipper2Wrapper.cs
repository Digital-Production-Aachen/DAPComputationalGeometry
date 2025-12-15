using Clipper2Lib;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using IntPoint = Clipper2Lib.Point64;
using IntZPoint = Clipper2ZLib.Point64;
using ClipZ = Clipper2ZLib;
using static System.MemoryExtensions;
using System.Collections.Generic;
using System.Threading;
using System.Linq;
using System;
using System.Numerics;

namespace g3
{
    /// <summary>
    /// Instanced clipper wrapper, more memory efficient for multiple clip operations. 
    /// </summary>
    public class Clipper2
    {
        private readonly Paths64 pathContainer = [];
        private Paths64 currentResult = [];
        private readonly Paths64 buffer;
        private readonly Clipper64 clipper64 = new();
        private readonly Path64 bufferPath = [];
        public bool PreserveCollinear { get { return clipper64.PreserveCollinear; }  set { clipper64.PreserveCollinear = value; } }
        public int BufferedVertexCount => buffer.Sum(x => x.Count);

        private static readonly ThreadLocal<Clipper2> reusableInstance = new();
        
        public static Clipper2 ThreadLocalInstance
        {
            get
            {
                if (!reusableInstance.IsValueCreated)
                    reusableInstance.Value = new Clipper2();

                reusableInstance.Value.Clear();
                return reusableInstance.Value;
            }
        }

        public Clipper2(int capacity = 0)
        {
            buffer = new(capacity);
            clipper64.PreserveCollinear = false;
        }

        public Clipper2(IEnumerable<Polygon2d> polygons)
        {
            foreach (var poly in polygons) AddPolygonSubject(poly);
        }

        public Clipper2(IEnumerable<GeneralPolygon2d> polygons)
        {
            foreach (var poly in polygons) AddPolygonSubject(poly);
        }

        public Clipper2(GeneralPolygon2d polygon)
        {
            AddPolygonSubject(polygon);
        }

        public Clipper2(Polygon2d polygon)
        {
            AddPolygonSubject(polygon);
        }

        public void AddPolygonSubject(Polygon2d polygon)
        {
            AddSubject(polygon);
        }

        public void AddPolygonSubject(Paths64 paths)
        {
            foreach (var path in paths) AddSubject(path);
        }

        public void AddPolygonSubject(Path64 path) => AddSubject(path);

        public void AddPolygonSubject(IEnumerable<Polygon2d> polygons)
        {
            foreach (var polygon in polygons)
                AddSubject(polygon);
        }

        public void AddPolygonSubject(GeneralPolygon2d polygon)
        {
            AddSubject(polygon.Outer);
            foreach (var poly in polygon.Holes)
                AddSubject(poly);
        }

        public void AddPolygonSubject(IEnumerable<GeneralPolygon2d> polygons)
        {
            foreach (var poly in polygons)
            {
                AddSubject(poly.Outer);
                foreach (var hole in poly.Holes)
                    AddSubject(hole);
            }
        }

        public void AddPolygonClip(Polygon2d polygon)
        {
            AddClip(polygon);
        }

        public void AddPolygonClip(IEnumerable<Polygon2d> polygons)
        {
            foreach (var polyline in polygons)
                AddClip(polyline);
        }

        public void AddPolygonClip(Paths64 paths)
        {
            foreach (var path in paths) AddClip(path);
        }

        public void AddPolygonClip(Path64 path) => AddClip(path);

        public void AddPolygonClip(GeneralPolygon2d polygon)
        {
            AddClip(polygon.Outer);
            foreach (var poly in polygon.Holes)
                AddClip(poly);
        }

        public void AddPolygonClip(IEnumerable<GeneralPolygon2d> polygons)
        {
            foreach (var poly in polygons)
            {
                AddClip(poly.Outer);
                foreach (var hole in poly.Holes)
                    AddClip(hole);
            }
        }

        private void AddSubject(Polygon2d subject)
        {
            Clipper2Wrapper.SetPathVertices(subject, bufferPath);
            AddSubject(bufferPath);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddSubject(Path64 path)
        {
            pathContainer.Add(path);
            clipper64.AddSubject(pathContainer);
            pathContainer.Clear();
        }

        private void AddClip(Polygon2d clip)
        {
            Clipper2Wrapper.SetPathVertices(clip, bufferPath);
            AddClip(bufferPath);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddClip(Path64 path)
        {
            pathContainer.Add(path);
            clipper64.AddClip(pathContainer);
            pathContainer.Clear();
        }

        public void AddOpenSubject(PolyLine2d polyline)
        {
            pathContainer.Add(polyline.ToIntPoints());
            clipper64.AddOpenSubject(pathContainer);
            pathContainer.Clear();
        }

        public void AddOpenSubject(IEnumerable<PolyLine2d> polylines)
        {
            foreach (var polyline in polylines)
                pathContainer.Add(polyline.ToIntPoints());
            clipper64.AddOpenSubject(pathContainer);
            pathContainer.Clear();
        }

        public void Union(FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            clipper64.Execute(ClipType.Union, fillRule, currentResult);
            buffer.AddRange(currentResult);
            clipper64.Clear();
        }

        public bool Execute(ClipType clipType, FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            bool success = clipper64.Execute(clipType, fillRule, currentResult);
            buffer.AddRange(currentResult);
            clipper64.Clear();
            return success;
        }

        public List<GeneralPolygon2d> UnionAsGenPoly(FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            var solution = new PolyTree64();
            clipper64.Execute(ClipType.Union, fillRule, solution);
            var genPolys = Clipper2Wrapper.FlattenTreeToGenPoly(solution);
            clipper64.Clear();
            return genPolys;
        }

        public void Difference(FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            clipper64.Execute(ClipType.Difference, fillRule, currentResult);
            buffer.AddRange(currentResult);
            clipper64.Clear();
        }

        public void Intersection(FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            clipper64.Execute(ClipType.Intersection, fillRule, currentResult);
            buffer.AddRange(currentResult);
            clipper64.Clear();
        }

        public void Xor(FillRule fillRule = Clipper2Wrapper.defaultFillRule)
        {
            clipper64.Execute(ClipType.Xor, fillRule, currentResult);
            buffer.AddRange(currentResult);
            clipper64.Clear();
        }

        public void FlushBufferAddAsSubjects()
        {
            clipper64.AddSubject(buffer);
            buffer.Clear();
        }

        public void FlushBufferAddAsSubjects(Clipper2 resultToFlush)
        {
            clipper64.AddSubject(resultToFlush.buffer);
            resultToFlush.buffer.Clear();
        }

        public void FlushBufferAddAsClip()
        {
            clipper64.AddClip(buffer);
            buffer.Clear();
        }

        public void FlushBufferAddAsClip(Clipper2 resultToFlush)
        {
            clipper64.AddClip(resultToFlush.buffer);
            resultToFlush.buffer.Clear();
        }

        public void Clear()
        {
            clipper64.Clear();
            buffer.Clear();
            currentResult.Clear();
            pathContainer.Clear();
        }

        public Paths64 TakeAndClearIntSolution()
        {
            //transfer ownership of solution
            var temp = currentResult;
            currentResult = new();
            return temp;
        }

        public Polygon2d[] Solution
        {
            get
            {
                if (currentResult.Count == 0) return [];
                var solution = new Polygon2d[currentResult.Count];
                for (int i = 0; i < currentResult.Count; i++)
                {
                    solution[i] = currentResult[i].ToPolygon2d();
                }
                return solution;
            }
        }
    }

    public class IntPointEqualityComparer : IEqualityComparer<IntPoint>
    {
        public bool Equals(IntPoint thisP, IntPoint otherP)
        {
            return thisP.X == otherP.X &&
                   thisP.Y == otherP.Y;
        }

        public int GetHashCode(IntPoint point)
        {
            int hashCode = 1861411795;
            hashCode = hashCode * -1521134295 + point.X.GetHashCode();
            hashCode = hashCode * -1521134295 + point.Y.GetHashCode();
            return hashCode;
        }
    }

    public static class Clipper2Wrapper
    {
        /// <summary>
        /// Scaling factor to convert floats to 64bit integers
        /// </summary>
        internal const int DefaultScale = 1 << 20;

        internal static AxisAlignedBox2d CoordinateMax 
        {
            get
            {
                const double max = long.MaxValue / Clipper2Wrapper.DefaultScale;
                const double min = long.MinValue / Clipper2Wrapper.DefaultScale;
                return new AxisAlignedBox2d(new Vector2d(min, min), new Vector2d(max, max));
            }
        }

        internal const FillRule defaultFillRule = FillRule.NonZero;

        /// <summary>
        /// Compute outer No-Fit Polygon (NFP): move one path with a first point as a reference point around other NFP
        /// NFP is a track of the first point in moving path
        /// </summary>
        /// <param name="fixedPath"></param>
        /// <param name="movingPath"></param>
        /// <returns></returns>
        public static Polygon2d GetOuterNfp(Polygon2d fixedPolygon, Polygon2d movingPolygon)
        {
            var pattern = fixedPolygon.ToIntPoints();
            var path = movingPolygon.ToIntPoints();

            //rotate moving polygon by 180° to get the minkowski difference
            for (int i = 0; i < path.Count; i++)
            {
                long X = path[i].X;
                long Y = path[i].Y;
                IntPoint intPoint = path[i];
                intPoint.X = (-1 * X);
                intPoint.Y = (-1 * Y);
                path[i] = intPoint;
            }

            Paths64 solution = Clipper.MinkowskiSum(pattern, path, true);
            var clipperNfp = new Path64();

            if (solution.Count == 0)
            {
                throw new Exception("Could not compute minkowski sum");
            }

            if (solution.Count > 1)
            {
                solution.OrderBy(poly => Clipper.Area(poly));
            }
            var nfp = solution[0].ToPolygon2d();

#if DEBUG
            //g3.SVGWriter sVGWriter = new g3.SVGWriter();
            //for (int i = 0; i < solution.Count; i++)
            //{
            //    var poly = solution[i];
            //    if (i == 0)
            //    {
            //        sVGWriter.AddPolygon(poly.ToPolygon2d(), g3.SVGWriter.Style.Outline("orange", 0.1f));
            //    }
            //    else
            //    {
            //        sVGWriter.AddPolygon(poly.ToPolygon2d(), g3.SVGWriter.Style.Outline("cyan", 0.1f));
            //    }
            //}
            //sVGWriter.AddPolygon(fixedPolygon, g3.SVGWriter.Style.Outline("magenta", 0.1f));
            //sVGWriter.AddPoint((float)fixedPolygon.Vertices.First().x, (float)fixedPolygon.Vertices.First().y, g3.SVGWriter.Style.Outline("magenta", 0.1f));
            //sVGWriter.AddPolygon(movingPolygon, g3.SVGWriter.Style.Outline("blue", 0.1f));
            //sVGWriter.AddPoint((float)movingPolygon.Vertices.First().x, (float)movingPolygon.Vertices.First().y, g3.SVGWriter.Style.Outline("blue", 0.1f));
            //sVGWriter.AddPoint(0, 0, g3.SVGWriter.Style.Outline("red", 0.1f));

            //sVGWriter.AddPolygon(nfp, g3.SVGWriter.Style.Outline("red", 0.1f));
            //sVGWriter.AddPoint((float)nfp.Vertices.First().x, (float)nfp.Vertices.First().y, g3.SVGWriter.Style.Outline("red", 0.1f));
            //string filename = System.IO.Path.GetTempPath() + $"multiple_NFP.svg";
            //sVGWriter.Write(filename);
#endif

            return nfp;
        }

        /// <summary>
        /// Aggregate a collection of polygons based on a distance tolerance. Polygon edges closer than distance to each other
        /// will be merged, gaps and concavities (approximately) smaller than tolerance will be closed (concave hull).
        /// The specified tolerance may be negative.
        /// The solution might contain holes (previous concavities larger than tolerance that are insulated when closing a gap).
        /// Holes might be degenerate/very small, filter using minHoleSize is recommended.
        /// Internally all outer polygons are offset by tolerance/2, unionized and then mapped back to the original contour.
        /// GenPoly Holes are not changed, but resorted to merged outers.
        /// </summary>
        /// <returns></returns>
        /// <exception cref="ApplicationException">thrown when Exception in Clipper occurs</exception>
        public static List<GeneralPolygon2d> Aggregate(
            this IEnumerable<GeneralPolygon2d> genPolys,
            double aggregationTolerance,
            double arcTolerance = 0.1, double miterLimit = 2.0,
            bool allowVertexIncreasingCords = false,
            bool allowInputSideEffects = false)
        {
            bool zeroZPointDetected = false;
            double offset = 0.5 * aggregationTolerance;

            // POLYGON PREPROCESSING
            // polygon simplification
            double simplifyTol, pointIdentityTol;
            (arcTolerance, simplifyTol, pointIdentityTol) = CalculateOffsetTolerances(offset, arcTol: arcTolerance);
            if (!allowInputSideEffects) genPolys.Clone();
            genPolys.SimplifyShifted(pointIdentityTol, simplifyTol);
            genPolys = genPolys.Union();

            var vtxCount = genPolys.Sum(p => p.VertexCount);
            var zMapping = new ZMapping((int)(vtxCount * 1.1), offset < 0);
            var joinType = ClipZ.JoinType.Round;
            var endType = ClipZ.EndType.Polygon;

            // convert polygons to clipper path types and assign each point a unique z value
            foreach (var genPoly in genPolys)
            {
                zMapping.AddPolygon(genPoly.Outer);
                foreach (var hole in genPoly.Holes)
                {
                    // clipper "fix": ignore holes that are guaranteed to disappear
                    // based on bounding box -> does not catch all
                    if (hole.Bounds.MinDim < -2 * offset) continue;
                    zMapping.AddPolygon(hole);
                }
            }

            // paths have to be added to clipper at once, otherwise the offset direction
            // of holes will be wrong
            var clipperOffset = new ClipZ.ClipperOffset(miterLimit * DefaultScale, arcTolerance * DefaultScale);
            clipperOffset.AddPaths(zMapping.paths, joinType, endType);

            // OFFSET
            // assign callback for handling intersection points, then execute clipper offset
            clipperOffset.ZCallback = zMapping.ZCallback;
            clipperOffset.PreserveCollinear = true;
            var offsetPaths = new ClipZ.Paths64();
            clipperOffset.Execute(offset * DefaultScale, offsetPaths);

#if DEBUG
            // for graphical debugging
            var offsetPolys = offsetPaths.ToPolygon2ds();
            var markers = new List<Vector2d>();
            foreach (var path in offsetPaths)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    if (path[i].Z == 0) markers.Add(path[i].ToVector2d());
                }
            }
#endif

            // BACKMAPPING
            // build the aggregated output by mapping the z values back to the original polygons
            // we expect one output polygon per offset solution polygon
            var backmappedPaths = new ClipZ.Paths64();
            foreach (var path in offsetPaths)
            {
                var backmappedPath = new ClipZ.Path64();
                ZMapping.PointData lastAdded = null;
                for (int i = 0; i < path.Count; i++)
                {
                    var point = path[i];
                    if (point.Z == 0) { zeroZPointDetected = true; }
                    else if (point.Z == long.MinValue) { zeroZPointDetected = true; }

                    // point directly derived from a point in the original polygon
                    // -> add the original point to the solution
                    else if (point.Z > 0)
                    {
                        // retrieve the original point
                        var oPointData = zMapping.GetOriginalPointData(point.Z);
                        int idx = oPointData.origPointIdx;
                        // filter out repeated points (happens at rounded corners of offset)
                        if (lastAdded != null && lastAdded == oPointData) continue;
                        backmappedPath.Add(oPointData.origPath[idx]);
                        lastAdded = oPointData;
                    }
                    // point caused by intersection
                    // -> linearly interpolate between the neighbors of the intersection point
                    else if (point.Z < 0)
                    {
                        var iPointData = zMapping.GetIntersectionPointData(point.Z);
                        // line 1
                        var bot1Data = zMapping.GetOriginalPointData(iPointData.bot1Z);
                        var top1Data = zMapping.GetOriginalPointData(iPointData.top1Z);
                        var b1 = bot1Data.origPath[bot1Data.origPointIdx];
                        var t1 = top1Data.origPath[top1Data.origPointIdx];
                        // line 2
                        var bot2Data = zMapping.GetOriginalPointData(iPointData.bot2Z);
                        var top2Data = zMapping.GetOriginalPointData(iPointData.top2Z);
                        var b2 = bot2Data.origPath[bot2Data.origPointIdx];
                        var t2 = top2Data.origPath[top2Data.origPointIdx];
                        //
                        int numSkipped = zMapping.CalculateNumSkippedVertices(iPointData);

                        switch (numSkipped, allowVertexIncreasingCords)
                        {
                            case (1, true):
                            case (2, true):
                            case ( < 0 or > 2, _): // connecting two different polygons or skipping more vertices
                                backmappedPath.Add(ZMapping.Lerp(b1, t1, iPointData.t1));
                                backmappedPath.Add(ZMapping.Lerp(b2, t2, iPointData.t2));
                                break;
                            case (1, false):
                                // if only one vertex would be skipped, that should be t1=b2, add that vertex
                                backmappedPath.Add(t1); break;
                            case (2, false):
                                // if two vertices would be skipped, these should be t1 and t2, so add them both
                                backmappedPath.AddRange([t1, b2]); break;
                        }
                    }
                }
                backmappedPaths.Add(backmappedPath);
            }
#if DEBUG
            var resultPolys = backmappedPaths.ToPolygon2ds();
#endif
            // sort polygon into hierarchy by doing a union operation
            var union = new ClipZ.Clipper64();
            var resultPaths = new ClipZ.PolyTree64();
            union.AddSubject(backmappedPaths);
            if (zeroZPointDetected)
            {
#if DEBUG
                string filePath = genPolys.ExportAsSVG();
                ILogging.Logger?.Warning($"point with Z=0 detected, exported to {filePath}");
#endif
                union.AddSubject(zMapping.paths);   // also add original polygons, to make sure they are fully contained
            }
            if (!union.Execute(ClipZ.ClipType.Union, ClipZ.FillRule.NonZero, resultPaths))
                throw new ApplicationException("clipper polygon operation failed");

            var result = FlattenTreeToGenPoly(resultPaths);
            return result;
        }

        private class ZMapping(int capacity, bool negativeOffset)
        {
            public bool NegativeOffset { get; init; } = negativeOffset;
            public List<Vector2d> intersectionPoints = new();
            public List<Vector2d> zeroZPoints = new();

            private long z1 = 1;    // count up for original points
            private long z2 = -1;   // and down for intersection points
            public Dictionary<long, PointData> zToPointData = new Dictionary<long, PointData>(capacity);
            public ClipZ.Paths64 paths = new ClipZ.Paths64();

            public OriginalPointData GetOriginalPointData(long z) { return (OriginalPointData)zToPointData[z]; }
            public IntersectionPointData GetIntersectionPointData(long z) { return (IntersectionPointData)zToPointData[z]; }

            public void AddPolygon(Polygon2d polygon, int scale = DefaultScale)
            {
                var path64 = new ClipZ.Path64(polygon.VertexCount);
                for (int i = 0; i < polygon.VertexCount; i++)
                {
                    var vertex = polygon.Vertices[i].ToIntZPoint(z1, scale);
                    var pointData = new OriginalPointData()
                    {
                        origPoly = polygon,
                        origPath = path64,
                        origPointIdx = i,
                    };
                    zToPointData.Add(z1, pointData);
                    path64.Add(vertex);
                    z1++;
                }
                paths.Add(path64);
            }

            public void ZCallback(IntZPoint bot1, IntZPoint top1, IntZPoint bot2, IntZPoint top2, ref IntZPoint point)
            {
                // BUG(?): Clipper can sometimes return points with Z = 0 (presumably on collinear points)
                // In this case there exists no backmapping
                if (bot1.Z <= 0 || top1.Z <= 0 || bot2.Z <= 0 || top2.Z <= 0)
                {
                    // Two possibilities:
                    // either log error and continue, then the point will be skipped (result will have missing points)
                    // or let it crash
                    //Log.Error("Point is not derived from an original point. Mapping this point back for interpolation will not be possible.");
                    intersectionPoints.Add(point.ToVector2d());
                    if (bot1.Z <= 0) zeroZPoints.Add(bot1.ToVector2d());
                    if (bot2.Z <= 0) zeroZPoints.Add(bot2.ToVector2d());
                    if (top1.Z <= 0) zeroZPoints.Add(top1.ToVector2d());
                    if (top2.Z <= 0) zeroZPoints.Add(top2.ToVector2d());
                    //throw new Exception($"Z value of point is 0. Mapping this point back for interpolation will not be possible.");
                    point.Z = long.MinValue;
                    return;
                }

                // sort points according to winding direction of the polygon
                SortSegmentByZ(ref bot1, ref top1);
                SortSegmentByZ(ref bot2, ref top2);

                // sort intersecting lines to ensure correct order
                if (!SegmentsInOrder(bot1, top1, bot2, top2) != NegativeOffset)
                {
                    (bot1, bot2) = (bot2, bot1);
                    (top1, top2) = (top2, top1);
                }

                // store all values needed for interpolation in the mapping
                double len1 = Math.Sqrt(Math.Pow(bot1.X - top1.X, 2) + Math.Pow(bot1.Y - top1.Y, 2));
                double len2 = Math.Sqrt(Math.Pow(bot2.X - top2.X, 2) + Math.Pow(bot2.Y - top2.Y, 2));
                double distToBot1 = Math.Sqrt(Math.Pow(bot1.X - point.X, 2) + Math.Pow(bot1.Y - point.Y, 2));
                double distToBot2 = Math.Sqrt(Math.Pow(bot2.X - point.X, 2) + Math.Pow(bot2.Y - point.Y, 2));
                var pointData = new IntersectionPointData()
                {
                    bot1Z = bot1.Z,
                    top1Z = top1.Z,
                    bot2Z = bot2.Z,
                    top2Z = top2.Z,
                    t1 = len1 > 0 ? distToBot1 / len1 : 0,
                    t2 = len2 > 0 ? distToBot2 / len2 : 0,
                };

                point.Z = z2--;
                zToPointData.Add(point.Z, pointData);
                Debug.Assert(point.Z != 0);
            }

            /// <summary>
            /// Calculate the number of vertices cut off by the cord that this intersection
            /// would create. Return -1 if the cord connects two different polygons.
            /// </summary>
            /// <param name="iPointData"></param>
            /// <returns></returns>
            public int CalculateNumSkippedVertices(IntersectionPointData iPointData)
            {
                // we make use of the fact that the lines/points in iPointData are ordered
                var firstSkippedData = (OriginalPointData)zToPointData[iPointData.top1Z];
                var lastSkippedData = (OriginalPointData)zToPointData[iPointData.bot2Z];

                if (firstSkippedData.origPoly != lastSkippedData.origPoly) return -1;

                int vertexCount = firstSkippedData.origPoly.VertexCount;
                int first = firstSkippedData.origPointIdx;
                int last = lastSkippedData.origPointIdx;
                if (last >= first) return last - first + 1;
                else return vertexCount - (last - first) + 1;   // rollover
            }

            public static IntZPoint Lerp(IntZPoint start, IntZPoint end, double t)
            {
                long x = (long)(start.X + t * (end.X - start.X));
                long y = (long)(start.Y + t * (end.Y - start.Y));
                return new IntZPoint() { X = x, Y = y };
            }

            /// <summary>
            /// Determine if two (directed) line segments are in clockwise order.
            /// </summary>
            private static bool SegmentsInOrder(IntZPoint g1, IntZPoint g2, IntZPoint h1, IntZPoint h2)
            {
                var v1 = new Vector3d(g2.X - g1.X, g2.Y - g1.Y, 0);
                var v2 = new Vector3d(h2.X - h1.X, h2.Y - h1.Y, 0);
                return (v1.Cross(v2).z < 0);
            }

            /// <summary>
            /// Sort two consecutive points of the same polygon by the order they appear in it.
            /// Make use of the fact that z-values are assigned incrementally.
            /// </summary>
            private static void SortSegmentByZ(ref IntZPoint p1, ref IntZPoint p2)
            {
                // points with consecutive indices
                if (Math.Abs(p1.Z - p2.Z) == 1) { if (p1.Z > p2.Z) (p1, p2) = (p2, p1); }
                // segment that closes the polygon
                else { if (p1.Z < p2.Z) (p1, p2) = (p2, p1); }
            }

            public abstract class PointData;
            public class OriginalPointData : PointData
            {
                public Polygon2d origPoly;
                public ClipZ.Path64 origPath;
                public int origPointIdx;
            }

            public class IntersectionPointData : PointData
            {
                public long bot1Z;
                public long top1Z;
                public long bot2Z;
                public long top2Z;
                public double t1;
                public double t2;
            }
        }        

        /// <summary>
        /// Retrieve a concave hull of the input polygon. Concavities and gaps with "entrances" approximately smaller than tolerance
        /// will be closed by chords.
        /// For clockwise/hole polygons this operation might create multiple polygons.
        /// Caution has to be taken if the polygon dimensions are similar to the tolerance as this might produce unexpected results.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="tolerance"></param>
        /// <param name="aggregationMode">REMOVE_CONCAVITIES or KEEP_CONVEX_CONCAVITIES, AGGREGATION_ONLY will do nothing for a single polygon</param>
        /// <returns></returns>
        public static Polygon2d[] ConcaveHull(this Polygon2d polygon, double tolerance)
        {
            var result = Aggregate([new GeneralPolygon2d(polygon)], tolerance, allowVertexIncreasingCords: true, allowInputSideEffects: true);
            return result.Select(x => x.Outer).ToArray();
        }

        /// <summary>
        /// Compute area union of polygons.
        /// </summary>
        /// <param name="polygons"></param>
        /// <returns></returns>
        public static Polygon2d[] Union(this IEnumerable<Polygon2d> polygons, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree(null, ClipType.Union, useThreadLocal, polygons, fillRule: fillRule);
        }

        //this has a different name because the Linq Union method has the same signature
        public static Polygon2d[] UnionPoly(this IEnumerable<Polygon2d> polygons, IEnumerable<Polygon2d> morePolygons, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree(null, ClipType.Union, useThreadLocal, polygons.Concat(morePolygons), fillRule: fillRule);
        }

        public static Polygon2d[] Union(this IEnumerable<Polygon2d> polygons, Polygon2d other, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree(null, ClipType.Union, useThreadLocal, polygons.Append(other), fillRule: fillRule);
        }

        public static Polygon2d[] RemoveSelfIntersections(this Polygon2d polygon, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree(null, ClipType.Union, useThreadLocal, [polygon], fillRule: fillRule);
        }

        public static Polygon2d[] Union(this Polygon2d polygon, Polygon2d other, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree(null, ClipType.Union, useThreadLocal, [polygon, other], fillRule: fillRule);
        }

        /// <summary>
        /// Clips subject by removing all areas occupied by polygonsToClip.
        /// </summary>
        /// <param name="subject"></param>
        /// <param name="polygonsToClip"></param>
        /// <returns></returns>
        public static Polygon2d[] ClipBy(this IEnumerable<Polygon2d> subject, IEnumerable<Polygon2d> polygonsToClip, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygonsToClip, ClipType.Difference, useThreadLocal, subject);
        }

        /// <summary>
        /// Clips subjects by removing all areas occupied by polygonToClip.
        /// </summary>
        /// <param name="subjects"></param>
        /// <param name="polygonToClip"></param>
        /// <returns></returns>
        public static Polygon2d[] ClipBy(this IEnumerable<Polygon2d> subjects, Polygon2d polygonToClip, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree([polygonToClip], ClipType.Difference, useThreadLocal, subjects);
        }

        /// <summary>
        /// Clips subject by removing all areas occupied by polygonsToClip.
        /// </summary>
        /// <param name="subject"></param>
        /// <param name="polygonsToClip"></param>
        /// <returns></returns>
        public static Polygon2d[] ClipBy(this Polygon2d subject, IEnumerable<Polygon2d> polygonsToClip, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygonsToClip, ClipType.Difference, useThreadLocal, [subject]);
        }

        public static Polygon2d[] ClipBy(this Polygon2d subject, IEnumerable<GeneralPolygon2d> polygonsToClip, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygonsToClip.OuterAndHolesItr(), ClipType.Difference, useThreadLocal, [subject]);
        }

        /// <summary>
        /// Clips subject by removing all areas occupied by polygonsToClip.
        /// </summary>
        /// <param name="subject"></param>
        /// <param name="polygonToClip"></param>
        /// <returns></returns>
        public static Polygon2d[] ClipBy(this Polygon2d subject, Polygon2d polygonToClip)
        {
            return subject.ClipBy([polygonToClip]);
        }

        public static List<GeneralPolygon2d> ClipBy(this GeneralPolygon2d subject, IEnumerable<GeneralPolygon2d> polygonsToClip, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Difference, fillRule, polygonsToClip);
        }

        public static List<GeneralPolygon2d> ClipBy(this GeneralPolygon2d subject, GeneralPolygon2d polygonToClip, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Difference, fillRule, [polygonToClip]);
        }

        public static Polygon2d[] XOr(this IEnumerable<Polygon2d> polygons, Polygon2d subject, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygons, ClipType.Xor, useThreadLocal, [subject]);
        }

        public static Polygon2d[] XOr(this IEnumerable<Polygon2d> polygons, IEnumerable<Polygon2d> subjects, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygons, ClipType.Xor, useThreadLocal, subjects);
        }

        public static Polygon2d[] XOr(this Polygon2d thisPolygon, Polygon2d polygon, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree([thisPolygon], ClipType.Xor, useThreadLocal, [polygon]);
        }

        public static List<GeneralPolygon2d> XOr(this GeneralPolygon2d subject, IEnumerable<GeneralPolygon2d> polygonsToClip, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Xor, fillRule, polygonsToClip);
        }

        public static List<GeneralPolygon2d> XOr(this GeneralPolygon2d subject, GeneralPolygon2d polygonToClip, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Xor, fillRule, [polygonToClip]);
        }

        public static Polygon2d[] Intersection(this Polygon2d thisPolygon, IEnumerable<Polygon2d> polygonsToClip, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree([thisPolygon], ClipType.Intersection, useThreadLocal, polygonsToClip);
        }

        public static Polygon2d[] Intersection(this IEnumerable<Polygon2d> polygons, Polygon2d subject, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygons, ClipType.Intersection, useThreadLocal, [subject]);
        }

        public static Polygon2d[] IntersectionPoly(this IEnumerable<Polygon2d> polygons, IEnumerable<Polygon2d> subjects, bool useThreadLocal = false)
        {
            return ClipperOperationNoTree(polygons, ClipType.Intersection, useThreadLocal, subjects);
        }

        public static Polygon2d[] Intersection(this Polygon2d thisPolygon, Polygon2d polygon, bool useThreadLocal = false, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperationNoTree([thisPolygon], ClipType.Intersection, useThreadLocal, [polygon], fillRule: fillRule);
        }

        public static List<GeneralPolygon2d> Intersection(this GeneralPolygon2d subject, IEnumerable<GeneralPolygon2d> polygonsToClip, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Intersection, fillRule, polygonsToClip);
        }

        public static List<GeneralPolygon2d> Intersection(this GeneralPolygon2d subject, GeneralPolygon2d polygonToClip, FillRule fillRule = defaultFillRule)
        {
            return ClipperOperation([subject], ClipType.Intersection, fillRule, [polygonToClip]);
        }

        public static List<Vector2d> FindIntersectionPoints(this IEnumerable<Polygon2d> clip, IEnumerable<Polygon2d> subjects, IEnumerable<PolyLine2d> openSubjects = null, FillRule fillRule = FillRule.NonZero)
        {
            if (clip is null) throw new ArgumentNullException(nameof(clip));

            var intersectPts = new List<Vector2d>();
            if (subjects == null && openSubjects == null)
            {
                return intersectPts;
            }

            int capacity = subjects?.Sum(x => x.VertexCount) ?? 0 + openSubjects?.Sum(x => x.VertexCount) ?? 0;
            var intCoordSet = new HashSet<IntPoint>(capacity, new IntPointEqualityComparer());
            Clipper64 clipper = new();

            Paths64 clipInt = new Paths64(clip.Select(poly => poly.ToIntPoints()));
            clipper.AddClip(clipInt);
            if (subjects != null)
            {
                Paths64 subjectsInt = new Paths64(subjects.Select(x => x.ToIntPoints()));
                clipper.AddSubject(subjectsInt);
                foreach (var sub in subjectsInt)
                    foreach (var pt in sub)
                        intCoordSet.Add(pt);
            }
            if (openSubjects != null)
            {
                Paths64 openPaths = new Paths64(openSubjects.Select(x => x.ToIntPoints()));
                clipper.AddOpenSubject(openPaths);
                foreach (var sub in openPaths)
                    foreach (var pt in sub)
                        intCoordSet.Add(pt);
            }

            //add trivial intersections where coordinates are equal, can't distinguish that after clipping
            intersectPts.AddRange(clipInt.SelectMany(x => x).Where(pt => intCoordSet.Contains(pt)).Select(x => x.ToVector2d()));

            Paths64 solution = new Paths64();
            Paths64 solutionOpen = new Paths64();

            clipper.Execute(ClipType.Difference, fillRule, solution, solutionOpen);

            if (solution == null)
            {
                throw new ApplicationException("clipper polygon operation failed");
            }

            //add all points that were not present in the subjects before, they must be intersections
            intersectPts.AddRange(solution.SelectMany(x => x).Where(pt => !intCoordSet.Contains(pt)).Select(x => x.ToVector2d()));
            intersectPts.AddRange(solutionOpen.SelectMany(x => x).Where(pt => !intCoordSet.Contains(pt)).Select(x => x.ToVector2d()));

            return intersectPts;
        }

        private struct zInfos
        {
            public int layerNr;
            public int idx;
            public int layerNr2;
            public int idx2;
        }

        public sealed class Polygon2dLayerConnection
        {
            public Polygon2d connectionPoly;
            public int[] layer1PolyGroupIdcs;
            public int Layer1PolyIdx => layer1PolyGroupIdcs[0];
            public int[] layer2PolyIdcs;
            public int firstLayer2PolyIdx => layer2PolyIdcs[0];
            public bool IsOutsideLayer2 => layer2PolyIdcs.Length == 0;
        }

        public static List<Polygon2dLayerConnection> FindHoleConnectionsWith(this IList<Polygon2d> layer1Polygons, IList<Polygon2d> layer2Polygons, Dictionary<int, int[]> layer1PolyGroupIdcs = null)
        {
            List<Polygon2dLayerConnection> connections = new();
            if (!layer1Polygons.Any(p => p.IsHole)) { return connections; }

            //use z to map points back to their source polygon
            long z = 0;
            Dictionary<long, zInfos> zMapping = new();
            var clipperZ = new ClipZ.Clipper64();
            clipperZ.PreserveCollinear = false;
            clipperZ.DefaultZ = -1;
            clipperZ.ZCallback = (IntZPoint bot1, IntZPoint top1, IntZPoint bot2, IntZPoint top2, ref IntZPoint intersectPt) =>
            {
                intersectPt.Z = z;
                var poly1 = zMapping[bot1.Z];
                var poly2 = zMapping[bot2.Z];
                zMapping.Add(z++, new zInfos() { layerNr = poly1.layerNr, idx = poly1.idx, layerNr2 = poly2.layerNr, idx2 = poly2.idx });
            };

            int polyIdx = 0;
            foreach (Polygon2d poly in layer1Polygons) 
            {
                clipperZ.AddSubject(poly.ToIntZPoints(z));
                zMapping.Add(z++, new zInfos() { layerNr = 1, idx = polyIdx++ });
            }

            // build a polytree from layer1
            var polyTree = new ClipZ.PolyTree64();
            if (!clipperZ.Execute(ClipZ.ClipType.Union, ClipZ.FillRule.NonZero, polyTree))
            {
                throw new ApplicationException("clipper polygon operation failed");
            }
            var layer1tree = polyTree as ClipZ.PolyPath64;
            clipperZ.Clear();

            ClipZ.ReuseableDataContainer64 layer2container = new();
            for (int i = 0; i < layer2Polygons.Count; i++)
            {
                Polygon2d layer2poly = layer2Polygons[i];
                var intPoly = layer2poly.ToIntZPoints(z);
                layer2container.AddPaths([intPoly], ClipZ.PathType.Clip, isOpen: false);
                zMapping.Add(z++, new zInfos() { layerNr = 2, idx = i });
            }

            // we are searching for hole connections.
            // strip the top level and reverse all parts of the poly tree starting from the first hole level
            foreach(var obj in layer1tree)
            {
                var path = obj as ClipZ.PolyPath64;
                // has any childs that could be holes?
                if (path.Count == 0) continue;

                foreach(var childObj in path)
                {
                    var hole = childObj as ClipZ.PolyPath64;
                    Debug.Assert(hole.IsHole);
                    AddHoleConnections(clipperZ, zMapping, layer2container, hole, connections, layer2Polygons, layer1PolyGroupIdcs);
                }
            }
            return connections;
        }

        private static void AddHoleConnections(ClipZ.Clipper64 clipperZ, Dictionary<long, zInfos> zMapping, 
            ClipZ.ReuseableDataContainer64 layer2container, ClipZ.PolyPath64 child, List<Polygon2dLayerConnection> connections, IList<Polygon2d> layer2Polygons, Dictionary<int, int[]> layer1PolyGroupIdcs)
        {
            Stack<ClipZ.PolyPath64> toDoStack = null;
            int[] holeIdcs = new int[child.Count + 1];
            Array.Fill(holeIdcs, -1);//make sure to get idx out of bounds if not filled
            var holeIntPoly = child.Polygon;
            holeIntPoly.Reverse();
            long holeZ = holeIntPoly[0].Z;
            Debug.Assert(zMapping[holeZ].layerNr == 1);
            holeIdcs[0] = zMapping[holeZ].idx;
            if (holeIntPoly.Any(pt => pt.Z != holeZ))
            { throw new ArgumentException("layer1Polygons have intersections, please perform a polygon union operation before calling FindHoleConnectionsWith"); }
            clipperZ.AddSubject(holeIntPoly);

            int holeIdcsCnt = 1;
            foreach (var grandChildObj in child)
            {
                var grandChild = grandChildObj as ClipZ.PolyPath64;
                var grandChildPoly = grandChild.Polygon;
                holeIdcs[holeIdcsCnt++] = zMapping[grandChildPoly[0].Z].idx;
                grandChildPoly.Reverse();
                var span = CollectionsMarshal.AsSpan(grandChildPoly);
                for (int i = 0; i < span.Length; i++) { span[i].Z = holeZ; }
                clipperZ.AddSubject(grandChildPoly);
                                
                foreach (var pathToRecurse in grandChild)
                {
                    toDoStack ??= new();
                    toDoStack.Push(pathToRecurse as ClipZ.PolyPath64);
                }
            }

            if(layer1PolyGroupIdcs != null) layer1PolyGroupIdcs[holeIdcs[0]] = holeIdcs;
            clipperZ.AddReuseableData(layer2container);
            var polyTree = new ClipZ.PolyTree64();
            clipperZ.Execute(ClipZ.ClipType.Difference, ClipZ.FillRule.NonZero, polyTree);
            var solution = polyTree as ClipZ.PolyPath64;

            foreach (var obj in solution)
            {
                var path = obj as ClipZ.PolyPath64;
                AddSolutionNode(zMapping, connections, layer2Polygons, path, holeIdcs);
            }

            clipperZ.Clear();
            while(toDoStack?.Count > 0) { AddHoleConnections(clipperZ, zMapping, layer2container, toDoStack.Pop(), connections, layer2Polygons, layer1PolyGroupIdcs); }
        }

        private static void AddSolutionNode(Dictionary<long, zInfos> zMapping, List<Polygon2dLayerConnection> connections, IList<Polygon2d> layer2Polygons, ClipZ.PolyPath64 path, int[] holeIdcs)
        {
            var poly = path.Polygon;
            var zValues = poly.Select(p => p.Z).ToHashSet();
            var genPoly = new GeneralPolygon2d(poly.ToPolygon2d());

            Stack<ClipZ.PolyPath64> toDoStack = null;
            if (path.Count > 0)
            {
                foreach (var childObj in path)
                {
                    var childPath = childObj as ClipZ.PolyPath64;
                    genPoly.AddHole(childPath.Polygon.ToPolygon2d(), false, false);

                    foreach (var pathToRecurse in childPath)
                    {
                        toDoStack ??= new();
                        toDoStack.Push(pathToRecurse as ClipZ.PolyPath64);
                    }
                }
            }

            HashSet<int> layer1Polys = new();
            HashSet<int> layer2Polys = new();
            foreach (var zValue in zValues)
            {
                var value = zMapping[zValue];
                HashSet<int> zSet = value.layerNr == 1 ? layer1Polys : layer2Polys;
                zSet.Add(value.idx);
                if (value.layerNr2 > 0)
                {
                    zSet = value.layerNr2 == 1 ? layer1Polys : layer2Polys;
                    zSet.Add(value.idx2);
                }
            }
            layer2Polys.RemoveWhere(idx => !layer2Polygons[idx].IsHole);

            //check hole containment, if the connection polygon is fully contained inside another hole, they are connected
            //full hole containment is the only case where we get no intersections from clipper z callback
            if (layer2Polygons.Any(poly => poly.IsHole && poly.Area > genPoly.Outer.Area - 1e-3))
            {
                // in theory 1 point check should be enough because Clipper makes sure there are no intersections
                // however there are rounding errors that can cause edge cases, so we check the "middle" point of the polygon as well
                if (layer2Polygons.ContainedInAHole(genPoly.Outer[0], out int containingIdx, Constants.DEFAULT_LINE_IDENTITY_TOLERANCE)
                 || layer2Polygons.ContainedInAHole(genPoly.Outer[genPoly.Outer.VertexCount / 2], out containingIdx, Constants.DEFAULT_LINE_IDENTITY_TOLERANCE))
                {
                    layer2Polys.Add(containingIdx);
                }
            }

            var connection = new Polygon2dLayerConnection()
            {
                connectionPoly = genPoly.Outer,
                layer1PolyGroupIdcs = holeIdcs,
                layer2PolyIdcs = layer2Polys.Distinct().ToArray(),
            };
            connections.Add(connection);
            while (toDoStack?.Count > 0) { AddSolutionNode(zMapping, connections, layer2Polygons, toDoStack.Pop(), holeIdcs); }
        }

        private static bool ContainedInAHole(this IList<Polygon2d> polygons, Vector2d P, out int containingIdx, double epsilon = MathUtil.ZeroTolerance)
        {
            int nContained = 0;
            containingIdx = -1;
            List<(Polygon2d poly, int idx)> sortedOrder = new(polygons.Count);
            for (int i = 0; i < polygons.Count; i++)
            {
                sortedOrder.Add((polygons[i], i));
            }
            sortedOrder.SortByDescending(p => p.Item1.Area);
            foreach (var pair in sortedOrder)
            {
                var poly = pair.poly;
                if (poly.ContainsInclusive(P, epsilon))
                {
                    if (poly.IsHole)
                    {
                        nContained--;
                        containingIdx = pair.idx;
                    }
                    else
                    {
                        nContained++;
                        containingIdx = -1;
                    }
                }
            }
            return nContained == 0 && containingIdx != -1;
        }

        private static Polygon2d[] ClipperOperationNoTree(IEnumerable<Polygon2d> polygons, ClipType operationType, bool useThreadLocal, IEnumerable<Polygon2d> differenceSubject, IEnumerable<PolyLine2d> openSubjects = null, FillRule fillRule = defaultFillRule)
        {
            Paths64 solution;
            if (useThreadLocal)
            {
                var clipper = Clipper2.ThreadLocalInstance;
                if(polygons != null) clipper.AddPolygonClip(polygons);
                if (differenceSubject != null) clipper.AddPolygonSubject(differenceSubject);
                if (openSubjects != null) clipper.AddOpenSubject(openSubjects);
                bool success = clipper.Execute(operationType, fillRule);
                if (success) solution = clipper.TakeAndClearIntSolution();
                else solution = null;
                clipper.Clear();
            }
            else
            {
                Paths64 clip = polygons == null ? null : new Paths64(polygons.Select(poly => poly.ToIntPoints()));
                Paths64 subjects = differenceSubject == null ? null : new Paths64(differenceSubject.Select(x => x.ToIntPoints()));
                solution = Clipper.BooleanOp(operationType, subjects, clip, fillRule);
            }
                
            if (solution == null)
            {
                throw new ApplicationException("clipper polygon operation failed");
            }
            return solution.Select(x => x.ToPolygon2d()).ToArray();
        }

        public static List<GeneralPolygon2d> SortPolygons(this IEnumerable<Polygon2d> polygons, FillRule fillRule = FillRule.EvenOdd)
        {
            if (polygons.Count() == 1)
            {
                var poly = polygons.First();
                if (poly.IsHole)
                    poly.Reverse();
                return new List<GeneralPolygon2d>(1) { new GeneralPolygon2d(poly) };
            }
            Paths64 subj = new Paths64(polygons?.Select(poly => poly.ToIntPoints()));

            var clipper = new Clipper64();
            clipper.PreserveCollinear = false;
            foreach (var poly in subj) clipper.AddSubject(poly);
            var clipperSolution = new PolyTree64();

            if (!clipper.Execute(ClipType.Union, fillRule, clipperSolution))
            {
                throw new ApplicationException("clipper polygon operation failed");
            }

            return FlattenTreeToGenPoly(clipperSolution);
        }

        public static List<GeneralPolygon2d> Intersection(this IEnumerable<GeneralPolygon2d> polygons, IEnumerable<GeneralPolygon2d> clips, FillRule fillRule = defaultFillRule)
            => ClipperOperation(polygons, ClipType.Intersection, fillRule, clips);

        public static List<GeneralPolygon2d> ClipBy(this IEnumerable<GeneralPolygon2d> polygons, IEnumerable<GeneralPolygon2d> clips, FillRule fillRule = defaultFillRule)
            => ClipperOperation(polygons, ClipType.Difference, fillRule, clips);

        public static List<GeneralPolygon2d> Union(this IEnumerable<GeneralPolygon2d> polygons, FillRule fillRule = defaultFillRule)
            => ClipperOperation(polygons, ClipType.Union, fillRule, null);

        public static List<GeneralPolygon2d> XOr(this IEnumerable<GeneralPolygon2d> subjects, IEnumerable<GeneralPolygon2d> clips, FillRule fillRule = defaultFillRule)
            => ClipperOperation(subjects, ClipType.Xor, fillRule, clips);

        private static List<GeneralPolygon2d> ClipperOperation(IEnumerable<GeneralPolygon2d> subjects, ClipType clipType, FillRule fillRule, IEnumerable<GeneralPolygon2d> clips)
        {
            if (!(subjects?.Any() == true) && !(clips?.Any() == true))
                return new List<GeneralPolygon2d>();

            var outers = subjects?.Select(x => x.Outer.ToIntPoints());
            var holes = subjects?.SelectMany(x => x.Holes).Select(x => x.ToIntPoints());
            var clipOuters = clips?.Select(x => x.Outer.ToIntPoints());
            var clipHoles = clips?.SelectMany(x => x.Holes).Select(x => x.ToIntPoints());

            var clipper = new Clipper64();
            if (outers?.Any() == true) foreach (var outer in outers) clipper.AddSubject(outer);
            if (holes?.Any() == true) foreach (var hole in holes) clipper.AddSubject(hole);
            if (clipOuters?.Any() == true) foreach (var outer in clipOuters) clipper.AddClip(outer);
            if (clipHoles?.Any() == true) foreach (var hole in clipHoles) clipper.AddClip(hole);

            var clipperSolution = new PolyTree64();
            if (!clipper.Execute(clipType, fillRule, clipperSolution))
            {
                throw new ApplicationException("clipper polygon operation failed");
            }

            return FlattenTreeToGenPoly(clipperSolution);
        }

        internal static List<GeneralPolygon2d> FlattenTreeToGenPoly(PolyPath64 path, int scale = DefaultScale)
        {
            var allTheResult = new List<GeneralPolygon2d>();
            GeneralPolygon2d resultPoly = null;
            if (path.Polygon != null)
            {
                resultPoly = new GeneralPolygon2d(path.Polygon.ToPolygon2d(scale));
                allTheResult.Add(resultPoly);
            }
            foreach (var child in path)
            {
                var childPath = child as PolyPath64;
                if (childPath.IsHole)
                {
                    resultPoly.AddHole(childPath.Polygon.ToPolygon2d(scale), false, false);
                    foreach (var grandChild in childPath)
                        allTheResult.AddRange(FlattenTreeToGenPoly(grandChild as PolyPath64, scale));
                }
                else
                {
                    allTheResult.AddRange(FlattenTreeToGenPoly(childPath, scale));
                }
            }
            return allTheResult;
        }

        internal static List<GeneralPolygon2d> FlattenTreeToGenPoly(ClipZ.PolyPath64 path)
        {
            var allTheResult = new List<GeneralPolygon2d>();
            GeneralPolygon2d resultPoly = null;
            if (path.Polygon != null)
            {
                resultPoly = new GeneralPolygon2d(path.Polygon.ToPolygon2d());
                allTheResult.Add(resultPoly);
            }
            foreach (var child in path)
            {
                var childPath = child as ClipZ.PolyPath64;
                if (childPath.IsHole)
                {
                    resultPoly.AddHole(childPath.Polygon.ToPolygon2d(), false, false);
                    foreach (var grandChild in childPath)
                        allTheResult.AddRange(FlattenTreeToGenPoly(grandChild as ClipZ.PolyPath64));
                }
                else
                {
                    allTheResult.AddRange(FlattenTreeToGenPoly(childPath));
                }
            }
            return allTheResult;
        }

        public static List<GeneralPolygon2d> Offset(this IEnumerable<GeneralPolygon2d> genPolys, double offset, double tolerance = 0.01)
        {
            return genPolys
                .SelectMany(gp => gp.OuterAndHolesItr())
                .Offset(offset, tolerance)
                .SortPolygons();
        }

        public static List<GeneralPolygon2d> Offset(this GeneralPolygon2d genPoly, double offset, double tolerance = 0.01)
        {
            return genPoly
                .OuterAndHolesItr()
                .Offset(offset, tolerance)
                .SortPolygons();
        }

        /// <summary>
        /// Offset a polygon with a fixed distance. Positive offsets (outer offsets) will increase the polygons area,
        /// negative offsets (inner offsets) will decrease it.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> Offset(this IEnumerable<Polygon2d> polygons, double offset, double tolerance = 0.01)
        {
            var resultPolys = OffsetFastUnsafe(polygons, offset, JoinType.Round, arcTolerance: tolerance);
#if DEBUG
            //capture test data if the offset fails bounds checks
            if (resultPolys.Any() && offset != 0)
            {
                var oldBounds = polygons.Bounds();
                var newBounds = resultPolys.Bounds();
                Polygon2d[] shouldBeNothing;
                if (offset >= 0)
                {
                    oldBounds.Expand(Math.Abs(offset) * (1 + tolerance / 2));
                    shouldBeNothing = polygons.ClipBy(resultPolys);
                }
                else
                {
                    oldBounds.Contract(Math.Abs(offset) * (1 - tolerance / 2));
                    shouldBeNothing = resultPolys.ClipBy(polygons);
                }

                //tolerate small area artifacts that might be generated by ~180° spikes being deleted
                if (!oldBounds.Contains(newBounds) || (shouldBeNothing.Length > 0))//&& shouldBeNothing.Any(x => x.Area > 1e-5)))
                {
                    var polygonsToDebug = resultPolys.ToArray();
                    var stencilSize = 0.04f;
                    var writer = polygons.StartExportAsSVG(SVGWriter.Style.Outline("lightgrey", stencilSize));
                    writer.AddPolygons(resultPolys, SVGWriter.Style.Outline("red", stencilSize));
                    writer.AddPolygons(shouldBeNothing, SVGWriter.Style.Outline("darkred", stencilSize));
                    writer.AddBox(oldBounds, SVGWriter.Style.Outline("green", stencilSize));
                    writer.AddBox(newBounds, SVGWriter.Style.Outline("orange", stencilSize));
                    var result = writer.WriteDebug();
                    polygons.ExportBin();
                    if (!oldBounds.Contains(newBounds))
                        ILogging.Logger?.Error($"polygon offset out of bounds, debug svg {result}");
                    else
                        ILogging.Logger?.Warning($"polygon offset artifacts detected, debug svg {result}");
                }
            }
#endif
            return resultPolys;
        }

        /// <summary>
        /// Offset a polygon with a fixed distance. Positive offsets (outer offsets) will increase the polygons area,
        /// negative offsets (inner offsets) will decrease it.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> Offset(this Polygon2d polygon, double offset, double tolerance = 0.01) => Offset([polygon], offset, tolerance);

        /// <summary>
        /// Offset a polygon with a fixed distance. Positive offsets (outer offsets) will increase the polygons area,
        /// negative offsets (inner offsets) will decrease it. Directly uses underlying clipper
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> OffsetFastUnsafe(this Polygon2d polygon, double offset, JoinType joinType = JoinType.Miter, double miterLimit = 2.0, double arcTolerance = 0.1)
        {
            //quick fail if the offset deletes the polygon
            if (offset < 0 && polygon.Bounds.MinDim < -2 * offset)
            {
                return Enumerable.Empty<Polygon2d>();
            }
            else if (offset == 0)
            {
                return [polygon.Duplicate()];
            }
            else
            {
                return OffsetFastUnsafe([polygon], offset, joinType, miterLimit, arcTolerance);
            }
        }

        /// <summary>
        /// Offset a polygon with a fixed distance. Positive offsets (outer offsets) will increase the polygons area,
        /// negative offsets (inner offsets) will decrease it. Directly uses underlying clipper
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> OffsetFastUnsafe(this IEnumerable<Polygon2d> polygons, double offset, JoinType joinType = JoinType.Miter, double miterLimit = 2.0, double arcTolerance = 0.1)
        {
            if (polygons is null || !polygons.Any())
                return Array.Empty<Polygon2d>();
            //be consistent and also copy the polygons if nothing is offset
            if (offset == 0)
                return polygons.Select(x => x.Duplicate());

            double simplifyTol, pointIdentityTol;
            (arcTolerance, simplifyTol, pointIdentityTol) = CalculateOffsetTolerances(offset, arcTolerance);
            var cleanPolys = polygons.CloneAndSimplify(pointIdentityTol, simplifyTol);

            // eliminate small polys that shrink. This does not cover all cases because it uses axis aligned bounds.
            for (int i = cleanPolys.Count - 1; i >= 0; i--)
            {
                Polygon2d poly = cleanPolys[i];
                //poly shrinks?
                if (offset < 0 ^ poly.IsHole)
                {
                    if (poly.Bounds.MinDim < -2 * offset)
                    {
                        cleanPolys.RemoveAt(i);
                    }
                }
            }
            if (cleanPolys.Count == 0) { return cleanPolys; }

            miterLimit = Math.Min(miterLimit, Math.Abs(offset));
            var intPolys = new Paths64(cleanPolys.Select(poly => poly.ToIntPoints()));
            var co = new ClipperOffset(miterLimit * DefaultScale, arcTolerance * DefaultScale, preserveCollinear: false);
            co.AddPaths(intPolys, joinType, EndType.Polygon);
            var result = new Paths64();
            co.Execute(offset * DefaultScale, result);

            var resultPoly = result.Select(x => x.ToPolygon2d());
            return resultPoly;
        }

        private static (double newArcTol, double simplifyTol, double pointIdentityTol) CalculateOffsetTolerances(double offset, double arcTol)
        {
            double absOffsetTol = Math.Abs(offset) / 4;
            //0.01 is the clipper internal limit (not exposed in any way...)
            const double clipperMinTolerance = 0.0100000001;
            var maxTolerance = Math.Max(absOffsetTol, clipperMinTolerance);
            double newArcTol = Math.Clamp(arcTol, clipperMinTolerance, maxTolerance);
            //very short segments will result in errors calculating the normals by clipper
            //collapse edges that have less than 10 units of length
            double simplifyTol = Math.Min(absOffsetTol, newArcTol);
            double pointIdentityTol = 10d / DefaultScale;
            return (newArcTol, simplifyTol, pointIdentityTol);
        }

        public static IList<Vector2d> Intersection(this Polygon2d polygon, Line2d line)
        {
            return Intersection([polygon], line);
        }

        public static IList<Vector2d> Intersection(this IEnumerable<Polygon2d> polygons, Line2d line) => Intersection(polygons, [line]);

        public static IList<Vector2d> Intersection(this IEnumerable<Polygon2d> polygons, IEnumerable<Line2d> lines)
        {
            if (polygons == null)
                return new List<Vector2d>();

            var bounds = polygons.Bounds();
            bounds.Expand(0.1);

            Paths64 openSubjects = new Paths64();
            foreach (var line in lines)
            {
                var openLine = new Path64(2);
                var coordScale = line.Direction.Length * 2;

                var maxDirection = (Int64.MaxValue / DefaultScale - line.Origin.Length) / coordScale;
                var minDirection = (Int64.MinValue / DefaultScale + line.Origin.Length) / coordScale;
                var maxIntercept = line.Origin + line.Direction * maxDirection;
                var minIntercept = line.Origin + line.Direction * minDirection;

                Debug.Assert(!bounds.Contains(maxIntercept));
                Debug.Assert(!bounds.Contains(minIntercept));

                openLine.Add(maxIntercept.ToIntPoint());
                openLine.Add(minIntercept.ToIntPoint());
                openSubjects.Add(openLine);
            }

            Paths64 clip = new Paths64(polygons.Select(poly => poly.ToIntPoints()));

            Clipper64 clipper = new Clipper64();
            clipper.AddClip(clip);
            clipper.AddOpenSubject(openSubjects);

            Paths64 solutionClosed = new Paths64(0);
            Paths64 solutionOpen = new Paths64();
            clipper.Execute(ClipType.Difference, defaultFillRule, solutionClosed, solutionOpen);

            Debug.Assert(solutionClosed.Count == 0);

            var result = new List<Vector2d>();
            foreach (var openLine in solutionOpen)
            {
                foreach (var point in openLine)
                {
                    var vector = point.ToVector2d();
                    if (bounds.Contains(vector))
                        result.Add(vector);
                }
            }

            return result;
        }

        public static bool IsConvex(this Path64 polygon, double tolerance = MathUtil.Epsilonf)
        {
            var vertices = CollectionsMarshal.AsSpan(polygon);
            if (vertices.Length <= 3)
                return true;
            //do first wrap around already here
            IntPoint prev = vertices[^2];
            IntPoint center = vertices[^1];
            for (int i = 0; i < vertices.Length; i++)
            {
                IntPoint next = vertices[i];
                var cp = center - prev;
                var cn = center - next;
                double sign = cp.Perp().Dot(cn);

                if (sign <= tolerance)
                    return false;

                prev = center;
                center = next;
            }
            return true;
        }

        public static IntPoint Perp(this IntPoint point) => new IntPoint(point.Y, -point.X);
        public static double Dot(this IntPoint point, IntPoint point2) => point.X * point2.X + point.Y * point2.Y;
        // Cross‐product in 2D: a.x*b.y – a.y*b.x
        public static long Cross(IntPoint a, IntPoint b) => a.X * b.Y - a.Y * b.X;

        #region type conversion extensions

        /// <summary>
        /// Multiply all points by scale constant
        /// </summary>
        /// <param name="points"></param>
        /// <returns></returns>
        public static Path64 ToIntPoints(this Polygon2d polygon, int scale = DefaultScale)
        {
            var output = new Path64(polygon.VertexCount);
            if (polygon.VertexCount > 0) SetPathVertices(polygon, output, scale);
            return output;
        }

        public static Path64 ToIntPoints(this PolyLine2d polyline, int scale = DefaultScale)
        {
            var output = new Path64(polyline.VertexCount);
            SetPathVertices(polyline, output, scale);
            return output;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetPathVertices(Polygon2d poly, Path64 path, int scale = DefaultScale)
            => SetPathVertices(poly.VerticesAsReadOnlySpan, path, scale);
        public static void SetPathVertices(PolyLine2d polyline, Path64 path, int scale = DefaultScale)
            => SetPathVertices(polyline.VerticesAsReadOnlySpan, path, scale);

        public static void SetPathVertices(ReadOnlySpan<Vector2d> vertices, Path64 path, int scale = DefaultScale)
        {
            int n = vertices.Length;
            CollectionsMarshal.SetCount(path, n);
            Span<IntPoint> destPts = CollectionsMarshal.AsSpan(path);

            ReadOnlySpan<double> dbls = MemoryMarshal.Cast<Vector2d, double>(vertices);
            ReadOnlySpan<Vector<double>> inVecs = MemoryMarshal.Cast<double, Vector<double>>(dbls);

            int vecCount = inVecs.Length;
            var scaleVec = new Vector<double>(scale);

            Span<Vector<long>> outVecs = MemoryMarshal.Cast<IntPoint, Vector<long>>(destPts);

            for (int v = 0; v < vecCount; v++)
            {
                // multiply all lanes (x0,y0,x1,y1,...) and then convert to Int64 lanes
                outVecs[v] = Vector.ConvertToInt64(inVecs[v] * scaleVec);
            }

            // 4) Handle any leftover points (when 2*n isn't a multiple of Vector<double>.Count)
            int processedPoints = (vecCount * Vector<long>.Count) / 2;
            for (int i = processedPoints; i < n; i++)
            {
                double x = dbls[2 * i];
                double y = dbls[2 * i + 1];
                destPts[i] = new IntPoint((long)(x * scale), (long)(y * scale));
            }
        }

        public static Polygon2d[] ToPolygon2d(this Paths64 paths, int scale = DefaultScale)
        {
            Polygon2d[] output = new Polygon2d[paths.Count];
            for (int i = 0; i < paths.Count; i++)
            {
                output[i] = paths[i].ToPolygon2d();
            }
            return output;
        }

        public static List<GeneralPolygon2d> SortToGeneralPolygon2d(this Paths64 paths, int scale = DefaultScale, FillRule fillRule = FillRule.EvenOdd)
        {
            var clipper = new Clipper64();
            clipper.PreserveCollinear = false;
            clipper.AddSubject(paths);
            var clipperSolution = new PolyTree64();
            clipper.Execute(ClipType.Union, fillRule, clipperSolution);
            return FlattenTreeToGenPoly(clipperSolution, scale);
        }

        public static Polygon2d ToPolygon2d(this Path64 path, int scale = DefaultScale)
        {
            var output = SharedPolyPool.Rent(path.Count);
            Span<Vector2d> vertices = output.VerticesAsSpanWithCount(path.Count);
            for (int i = 0; i < path.Count; i++)
            {
                vertices[i] = path[i].ToVector2d(scale);
            }
            return output;
        }

        public static Polygon2d ToPolygon2d(this ClipZ.Path64 path, int scale = DefaultScale)
        {
            var output = SharedPolyPool.Rent(path.Count);
            Span<Vector2d> vertices = output.VerticesAsSpanWithCount(path.Count);
            for (int i = 0; i < path.Count; i++)
            {
                vertices[i] = path[i].ToVector2d(scale);
            }
            return output;
        }

        internal static List<Polygon2d> ToPolygon2ds(this ClipZ.Paths64 paths, int scale = DefaultScale)
        {
            var result = new List<Polygon2d>();
            foreach (var path in paths) result.Add(path.ToPolygon2d());
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ToVector2d(this IntPoint point, int scale = DefaultScale)
        {
            return new Vector2d((double)point.X / scale, (double)point.Y / scale);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static IntPoint ToIntPoint(this Vector2d vector, int scale = DefaultScale)
        {
            return new IntPoint((long)(vector.x * scale), (long)(vector.y * scale));
        }


        //--- methods for dealing with clipper points with additional user defined Z (not 3d vector) ---

        internal static ClipZ.Path64 ToIntZPoints(this Polygon2d polygon, long z = 0, int scale = DefaultScale)
        {
            return new ClipZ.Path64(polygon.Vertices.Select(x => x.ToIntZPoint(z, scale)));
        }

        internal static ClipZ.Path64 ToIntPointsIncrementZ(this Polygon2d polygon, ref long z, int scale = DefaultScale)
        {
            var path64 = new ClipZ.Path64(polygon.VertexCount);
            for (int i = 0; i < polygon.VertexCount; i++)
            {
                var vertex = polygon.Vertices[i];
                path64.Add(vertex.ToIntZPoint(z++, scale));
            }
            return path64;
        }

        internal static ClipZ.Path64 ToIntZPoints(this PolyLine2d polygon, long z = 0, int scale = DefaultScale)
        {
            return new ClipZ.Path64(polygon.Vertices.Select(x => x.ToIntZPoint(z, scale)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ToVector2d(this IntZPoint point, int scale = DefaultScale)
        {
            return new Vector2d((double)point.X / scale, (double)point.Y / scale);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static IntZPoint ToIntZPoint(this Vector2d vector, long z = 0, int scale = DefaultScale)
        {
            return new IntZPoint((long)(vector.x * scale), (long)(vector.y * scale), z);
        }

        public static Rect64 ToRect64(this AxisAlignedBox2d aabb, int scale = DefaultScale)
        {
            aabb.Scale(scale);
            return new Rect64((long)aabb.Min.x, (long)aabb.Min.y, (long)aabb.Max.x, (long)aabb.Max.y);
        }

        public static AxisAlignedBox2d ToAab2d(this Rect64 rect, int scale = DefaultScale)
        {
            var aabb = new AxisAlignedBox2d(rect.left, rect.top, rect.right, rect.bottom);
            aabb.Scale(1.0 / scale);
            return aabb;
        }

        public static Rect64 Bounds(this Paths64 paths) => Clipper.GetBounds(paths);

        public static Rect64 Bounds(this Path64 path) => Clipper.GetBounds(path);

        public static void Expand(this ref Rect64 rect, long extend)
        {
            rect.left -= extend;
            rect.right += extend;
            rect.top -= extend;
            rect.bottom += extend;
        }

        public static string ToClipperTestString(this IEnumerable<Path64> paths)
        {
            StringBuilder builder = new StringBuilder();
            foreach (var path in paths)
            {
                int ct = path.Count - 1;
                for (int i = 0; i < ct; i++)
                {
                    builder.Append($"{path[i].X},{path[i].Y} ");
                }
                builder.Append($"{path[ct].X},{path[ct].Y}\r\n");
            }
            return builder.ToString();
        }

        #endregion
    }
}