using Clipper2Lib;
using g3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace g3
{
    public sealed class SplitPolygons
    {
        public readonly List<PolyLine2d> positiveSide = new();
        public readonly List<PolyLine2d> negativeSide = new();
        internal void AddPolyline(PolyLine2d polyline, int side)
        {
            if (side == 1) positiveSide.Add(polyline);
            else negativeSide.Add(polyline);
        }
    }

    public static class Polygon2dExtensions
    {
        /// <summary>
        /// Returns the convex hull of this polygon, or the polygon itself if it is convex.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        public static Polygon2d ConvexHull(this Polygon2d polygon, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            if (polygon.IsConvex(epsilon))
                return polygon;
            ConvexHull2 convexHull = CreateConvexHull2(polygon, out _, epsilon);
            return convexHull.GetHullPolygon();
        }

        /// <summary>
        /// Returns the convex hull of the polygon using a linear time algorithm.
        /// This method must only be called one simple polygons without self intersections!
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        public static Polygon2d ConvexHullSimplePolygon(this Polygon2d polygon, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            if (polygon.IsConvex(epsilon))
                return polygon;
            return ConvexHull2Polygon.ConvexHullPoly(polygon, epsilon);
        }

        public static ConvexHull2 CreateConvexHull2(Polygon2d polygon, out IReadOnlyList<Vector2d> maybeMovedVertices, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            maybeMovedVertices = MoveSelfTouches(polygon, epsilon);
            return new ConvexHull2(maybeMovedVertices, epsilon);
        }

        public static Polygon2d ConvexHull(this IReadOnlyList<Vector2d> vertices, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            ConvexHull2 convexHull = new ConvexHull2(vertices, epsilon);
            return convexHull.GetHullPolygon();
        }

        private static IReadOnlyList<Vector2d> MoveSelfTouches(Polygon2d polygon, double moveDistance)
        {
            IReadOnlyList<Vector2d> vertices = polygon.Vertices;
            var vertexSet = new Dictionary<Vector2d, int>(vertices.Count);
            Vector2d[] copy = null;

            for (int i = 0; i < vertices.Count; i++)
            {
                int moveMulti = 1;
                while (!vertexSet.TryAdd(vertices[i], i))
                {
                    var otherVtxIdx = vertexSet[vertices[i]];
                    //copy the vertices only in rare case we have to change them
                    if (copy == null)
                    {
                        copy = new Vector2d[vertices.Count];
                        (polygon.Vertices as ICollection<Vector2d>).CopyTo(copy, 0);
                        vertices = copy;
                    }

                    //slightly move the two touching vertices
                    var convex1 = polygon.IsVertexConvex(i, -MathUtil.ZeroTolerance);
                    var convex2 = polygon.IsVertexConvex(otherVtxIdx, -MathUtil.ZeroTolerance);
                    var normal = polygon.GetNormal_FaceAvg(i);
                    var normal2 = polygon.GetNormal_FaceAvg(otherVtxIdx);
                    if (normal == Vector2d.Zero || normal2 == Vector2d.Zero)
                    {
                        //polygon has duplicate vertices
                        break;
                    }
                    if (convex1 == convex2)
                    {
                        copy[i] -= normal * moveDistance * moveMulti;
                        copy[otherVtxIdx] -= normal2 * moveDistance;
                    }
                    else
                    {
                        copy[i] += normal * moveDistance * moveMulti;
                        copy[otherVtxIdx] += normal2 * moveDistance;
                    }
                    moveMulti++;
                }
            }
            return copy ?? vertices;
        }

        public static Polygon2d ConvexHull(this IEnumerable<Polygon2d> polygons, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            using (var e = polygons.GetEnumerator())
            {
                if (!e.MoveNext())
                    return new Polygon2d();
                Polygon2d firstPoly = e.Current;
                if (!e.MoveNext())
                    return firstPoly.ConvexHull(epsilon);
            }

            var vtxCount = polygons.Sum(p => p.VertexCount);
            var vertices = new List<Vector2d>(vtxCount);
            foreach (var poly in polygons)
            {
                var maybeMovedVertices = MoveSelfTouches(poly, epsilon);
                vertices.AddRange(maybeMovedVertices);
            }
            ConvexHull2 convexHull = new ConvexHull2(vertices, epsilon);
            return convexHull.GetHullPolygon();
        }

        public static bool ConvexHullContains(this Polygon2d polygon, Vector2d point)
            => ConvexHullContains([polygon], point);

        /// <summary>
        /// Tests whether point is contained inside the convex hull of the polygon group or not.
        /// Fast linear time algorithm, avoids computing the convex hull (Quickhull) which has linearithmic 
        /// complexity for polygon groups.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static bool ConvexHullContains(this IEnumerable<Polygon2d> polygons, Vector2d point)
        {
            if(!(polygons?.Any(p => p.VertexCount > 0) == true)) return true;

            //init left and right extreme direction vectors (rays)
            Vector2d firstVertex = polygons.First(p => p.VertexCount > 0)[0];
            Vector2d leftDir = firstVertex - point;
            Vector2d rightDir = leftDir;
            foreach (var poly in polygons)
            {
                ReadOnlySpan<Vector2d> verts = poly.VerticesAsReadOnlySpan;                
                for (int i = 0; i < verts.Length; i++)
                {
                    Vector2d dir = verts[i] - point;

                    double crossLeft = leftDir.x * dir.y - leftDir.y * dir.x;
                    double crossRight = rightDir.x * dir.y - rightDir.y * dir.x;
                    bool isLeft = crossLeft > 0;
                    bool isRight = crossRight < 0;

                    //if both are true, viewPoint is within a concavity of the polygon [group]
                    //and the angle interval exceeds 180°
                    if (isLeft && isRight) return true;
                    if (isLeft) leftDir = dir;
                    if (isRight) rightDir = dir;
                }
            }
            return false;
        }

        public static bool CountIsOne<TSource>(this IEnumerable<TSource> source)
        {
            if (source is ICollection<TSource> collectionOfT)
            {
                return collectionOfT.Count == 1;
            }

            using IEnumerator<TSource> e = source.GetEnumerator();
            if (!e.MoveNext()) return false;
            return !e.MoveNext();            
        }

        public static Polygon2d ConvexHullSimplePolygons(this IEnumerable<Polygon2d> simplePolygons, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            Polygon2d outer = null;
            foreach (var poly in simplePolygons)
            {
                if (poly.IsHole) continue;
                if (outer != null)
                {
                    return simplePolygons.ConvexHull(epsilon);
                }
                outer = poly;
            }
            return outer?.ConvexHullSimplePolygon() ?? new Polygon2d();
        }

        public static Polygon2d ConvexHull(this IEnumerable<GeneralPolygon2d> polygons, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
            => polygons.Select(x => x.Outer).ConvexHull(epsilon);

        /// <summary>
        /// Tests whether point is contained inside a hole of the given list of polygons.
        /// This check assumes polygons do not intersect (use polygon union operations to ensure this)
        /// Returns the index of the innermost nested hole that contains point.
        /// Returns -1 instead if point is not contained inside a hole (or there are no holes).
        /// </summary>
        /// <param name="polygons">the list of polygons</param>
        /// <param name="point">point to test</param>
        /// <param name="tol">tolerance for inclusion checks</param>
        /// <returns></returns>
        public static int IndexOfContainingHole(this IReadOnlyList<Polygon2d> polygons, Vector2d point, double tol = MathUtil.ZeroTolerance)
        {
            //debug check, is a full union op, more expensive than this whole method
            Debug.Assert(!polygons.HaveIntersections());

            Polygon2d containingHole = null;
            int holeIdx = -1;
            for (int i = 0; i < polygons.Count; i++)
            {
                if (!polygons[i].IsHole) continue;
                Polygon2d hole = polygons[i];
                if (!hole.ContainsInclusive(point, tol)) { continue; }
                //if start is contained in multiple holes, they must be nested (because obstacles.HaveSelfIntersections() == false)
                //the nested hole must have a smaller area than the one it is nested in
                if (hole.Area > containingHole?.Area) continue;
                containingHole = hole;
                holeIdx = i;
            }
            return holeIdx;
        }

        /// <summary>
        /// Tests whether point is contained inside the given list of polygons.
        /// This check assumes polygons do not intersect (use polygon union operations to ensure this)
        /// Returns the index of the innermost nested polygon that contains point.
        /// Returns -1 instead if point is not contained inside any polygon.
        /// </summary>
        /// <param name="polygons">the list of polygons</param>
        /// <param name="point">point to test</param>
        /// <param name="tol">tolerance for inclusion checks</param>
        /// <returns></returns>
        public static int IndexOfContainingPoly(this IReadOnlyList<Polygon2d> polygons, Vector2d point, double tol = MathUtil.ZeroTolerance)
        {
            //debug check, is a full union op, more expensive than this whole method
            Debug.Assert(!polygons.HaveIntersections(allowPointContact: true));

            Polygon2d containingPoly = null;
            int polyIdx = -1;
            for (int i = 0; i < polygons.Count; i++)
            {
                Polygon2d poly = polygons[i];
                if (poly.IsHole)
                {
                    if (!poly.ContainsInclusive(point, tol)) continue;
                }
                else
                {
                    if (!poly.ContainsExclusive(point, tol)) continue;
                }

                //if start is contained in multiple holes, they must be nested (because polygons.HaveIntersections() == false)
                //the nested hole must have a smaller area than the one it is nested in
                if (poly.Area > containingPoly?.Area) continue;
                containingPoly = poly;
                polyIdx = i;
            }
            return polyIdx;
        }

        /// <summary>
        /// Tests whether P is contained inclusive (contained or on the boundary) in the union of a collection of polygons.
        /// Containment counts positive for outer and negative for hole polygons.
        /// </summary>
        public static bool ContainsInclusive(this IEnumerable<Polygon2d> polygons, Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            int containmentCount = 0;
            foreach(Polygon2d poly in polygons)
            {
                if(poly.ContainsInclusive(P, epsilon))
                {
                    if (poly.IsHole) containmentCount++;
                    else containmentCount--;
                }
            }
            return containmentCount != 0;
        }

        /// <summary>
        /// Tests whether P is contained exclusive (contained, but not on the boundary) in the union of a collection of polygons.
        /// Containment counts positive for outer and negative for hole polygons.
        /// </summary>
        public static bool ContainsExclusive(this IEnumerable<Polygon2d> polygons, Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            int containmentCount = 0;
            foreach (Polygon2d poly in polygons)
            {
                if (poly.ContainsExclusive(P, epsilon))
                {
                    if (poly.IsHole) containmentCount++;
                    else containmentCount--;
                }
            }
            return containmentCount != 0;
        }

        /// <summary>
        /// Test whether all of the given polygons are simple polygons or any of them has self intersections.
        /// Additionally tests whether any of the polygons intersect with each other.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="useThreadLocalClipper"></param>
        /// <returns></returns>
        public static bool HaveIntersections(this IReadOnlyCollection<Polygon2d> polygons, bool useThreadLocalClipper = false, bool allowPointContact = false)
            => HaveIntersections(polygons, out _, useThreadLocalClipper, allowPointContact);

        internal static bool HaveIntersections(this IReadOnlyCollection<Polygon2d> polygons, out Paths64 union, bool useThreadLocalClipper = false, bool allowPointContact = false)
        {
            if (polygons == null || polygons.Count == 0) { union = null; return false; }
            if (polygons.Count == 1) return HasSelfIntersections(polygons.First(), out union, useThreadLocalClipper);
            var clipper2 = useThreadLocalClipper ? Clipper2.ThreadLocalInstance : new Clipper2();
            bool preserveCollinearOrig = clipper2.PreserveCollinear;
            clipper2.PreserveCollinear = true;
            Paths64 input = new(polygons.Count);
            foreach (Polygon2d polygon in polygons)
            {
                //use a higher than the default scale to reduce issues from rounding round trips resulting in false positives here
                Path64 intPoly = polygon.ToIntPoints();
                input.Add(intPoly);
                clipper2.AddPolygonSubject(intPoly);
            }
            clipper2.Union(FillRule.NonZero);
            union = clipper2.TakeAndClearIntSolution();
            clipper2.PreserveCollinear = preserveCollinearOrig;
            clipper2.Clear();

            bool differs = !input.EqualsUnordered(union);
            if (!differs) return false;
            //clipper reverses the input if everything is holes, check reversed
            foreach (var intPoly in union) intPoly.Reverse();
            differs = !input.EqualsUnordered(union);
            if (!differs) return false;
            if (allowPointContact)
            {
                return IntersectionsCloseToVertex(union, input);
            }
            else
            {
                return true;
            }
        }

        /// <summary>
        /// Test whether all of the given polygons are simple polygons or any of them has self intersections.
        /// </summary>
        /// <param name="polygon">polygon to test</param>
        /// <returns></returns>
        public static bool HaveSelfIntersections(this IEnumerable<Polygon2d> polygons, bool useThreadLocalClipper = false)
            => polygons?.Any(p => p.HasSelfIntersections(useThreadLocalClipper: useThreadLocalClipper)) ?? false;

        /// <summary>
        /// Test whether the given polygon is a simple polygon or has self intersections
        /// </summary>
        /// <param name="polygon">polygon to test</param>
        /// <returns></returns>
        public static bool HasSelfIntersections(this Polygon2d polygon, bool useThreadLocalClipper = false, bool allowPointContact = false)
            => HasSelfIntersections(polygon, out _, useThreadLocalClipper, allowPointContact);

        internal static bool HasSelfIntersections(this Polygon2d polygon, out Paths64 union, bool useThreadLocalClipper = false, bool allowPointContact = false)
        {
            if (polygon == null || polygon.Vertices.Count <= 3) { union = null; return false; }
            var clipper2 = useThreadLocalClipper ? Clipper2.ThreadLocalInstance : new Clipper2();
            bool preserveCollinearOrig = clipper2.PreserveCollinear;
            clipper2.PreserveCollinear = true;
            Path64 intPoly = polygon.ToIntPoints();
            clipper2.AddPolygonSubject(intPoly);
            clipper2.Union(Clipper2Lib.FillRule.EvenOdd);
            union = clipper2.TakeAndClearIntSolution();
            clipper2.PreserveCollinear = preserveCollinearOrig;
            clipper2.Clear();

            if (union.Count != 1)
            {
                if (!allowPointContact) return true;
                return IntersectionsCloseToVertex(union, [intPoly]);
            }
            var result = union[0];
            if (polygon.IsHole) result.Reverse();
            return !EqualsPoly(intPoly, result);
        }

        private static bool IntersectionsCloseToVertex(Paths64 union, IEnumerable<Path64> intPolys)
        {
            HashSet<Point64> intPolySet = [.. intPolys.SelectMany(x => x)];
            HashSet<Point64> diff = new();
            foreach(var poly in union)
            {
                foreach(var point in poly)
                {
                    if (!intPolySet.Contains(point))
                        diff.Add(point);
                }
            }

            foreach(var intersectionP in diff)
            {
                var dist = intPolySet.Min(pt => Clipper.DistanceSqr(pt, intersectionP));
                double distUnscaled = Math.Sqrt(dist) / Clipper2Wrapper.DefaultScale;
                if (distUnscaled > Constants.DEFAULT_LINE_IDENTITY_TOLERANCE * 200)
                    return true;
            }

            return false;
        }

        public static bool EqualsUnordered(this Paths64 intPolyA, Paths64 intPolyB)
        {
            //quick check poly counts
            if (intPolyA.Count != intPolyB.Count)
                return false;
            //quick check sum of vertex counts
            if (intPolyA.Sum(p => p.Count) != intPolyB.Sum(p => p.Count))
                return false;

            //accept any order, but assume both lists have the same order
            for (int ia = 0; ia < intPolyA.Count; ia++)
            {
                Path64 polyA = intPolyA[ia];
                bool equalsAny = false;
                int ib = ia;
                do
                {
                    Path64 polyB = intPolyB[ib];
                    equalsAny |= polyA.EqualsPoly(polyB);
                    if (equalsAny) break;
                    ib = ib < intPolyB.Count - 1 ? ib + 1 : 0;
                } while (ib != ia);

                if (!equalsAny) return false;
            }
            return true;
        }

        public static bool EqualsPoly(this Path64 intPolyA, Path64 intPolyB)
        {
            if (ReferenceEquals(intPolyA, intPolyB)) return true;
            if (intPolyB.Count != intPolyA.Count) return false;

            int offset = 0;
            var firstVtx = intPolyA[0];
            for (int i = 0; i < intPolyB.Count; i++)
            {
                if (firstVtx == intPolyB[i])
                {
                    offset = i;
                    break;
                }
            }

            int cnt = intPolyB.Count;
            for (int i = 0; i < cnt; i++)
            {
                int offsetIdx = i + offset;
                offsetIdx = offsetIdx >= cnt ? offsetIdx - cnt : offsetIdx;
                if (intPolyB[offsetIdx] != intPolyA[i]) return false;
            }
            return true;
        }

        public static Box2d MinimalBoundingBox(this IEnumerable<Polygon2d> polygons, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            var convHull = polygons.ConvexHull(epsilon);
            return new ContMinBox2(convHull.Vertices, epsilon, QueryNumberType.QT_DOUBLE, isConvexPolygon: true).MinBox;
        }

        public static void Translate(this IEnumerable<Polygon2d> polygons, Vector2d translation)
        {
            foreach (var poly in polygons) poly.Translate(translation);
        }

        public static void Translate(this IEnumerable<GeneralPolygon2d> polygons, Vector2d translation)
        {
            foreach (var poly in polygons) poly.Translate(translation);
        }

        public static void Rotate(this IEnumerable<Polygon2d> polygons, Matrix2d rotation, Vector2d origin)
        {
            foreach (var poly in polygons) poly.Rotate(rotation, origin);
        }

        public static void TrimExcess(this IEnumerable<Polygon2d> polygons)
        {
            foreach (var poly in polygons) poly.TrimExcess();
        }

        public static IEnumerable<Polygon2d> Clone(this IEnumerable<g3.Polygon2d> polygons)
        {
            foreach (var poly in polygons)
                yield return poly.Duplicate();
        }

        public static Polygon2d[] DeepClone(this g3.Polygon2d[] polygons)
        {
            var copy = new Polygon2d[polygons.Length];
            for (int i = 0; i < polygons.Length; i++)
            {
                copy[i] = polygons[i].Duplicate();
            }
            return copy;
        }

        public static IEnumerable<GeneralPolygon2d> Clone(this IEnumerable<GeneralPolygon2d> polygons)
        {
            foreach (var poly in polygons)
                yield return new GeneralPolygon2d(poly);
        }

        public static Polygon2d[] DebugView(this GeneralPolygon2d polygon)
        {
            return polygon.OuterAndHolesItr().ToArray();
        }

        public static Polygon2d[] DebugView(this IEnumerable<GeneralPolygon2d> polygons)
        {
            return polygons.SelectMany(x => x.OuterAndHolesItr()).ToArray();
        }

        public static Vector2d Centroid(this IEnumerable<Polygon2d> polygons)
        {
            Vector2d centroid = Vector2d.Zero;
            double totalArea = 0;
            foreach (var poly in polygons)
            {
                var polyCentroid = poly.Centroid();
                var signedArea = poly.SignedArea;
                totalArea += signedArea;
                centroid += polyCentroid * signedArea;
            }
            centroid /= totalArea;
            return centroid;
        }

        /// <summary>
        /// Compute the centroid (center of mass) of a point set
        /// </summary>
        /// <param name="vectors"></param>
        /// <returns></returns>
        public static Vector2d Centroid(this IEnumerable<Vector2d> vectors)
        {
            Vector2d centroid = Vector2d.Zero;
            int count = 0;
            foreach (Vector2d v in vectors)
            {
                centroid.x += v.x;
                centroid.y += v.y;
                count++;
            }
            if(count > 0) centroid /= count;
            return centroid;
        }

        public static Vector2d Centroid(this ReadOnlySpan<Vector2d> vectors)
        {
            Vector2d centroid = Vector2d.Zero;
            foreach (Vector2d v in vectors)
            {
                centroid += v;
            }
            if(vectors.Length > 0) centroid /= vectors.Length;
            return centroid;
        }

        public static double MinCoord(this Vector2d vec) => vec.x < vec.y ? vec.x : vec.y;
        public static double MaxCoord(this Vector2d vec) => vec.x > vec.y ? vec.x : vec.y;
        public static Vector2d Abs(this Vector2d vec) => new Vector2d(Math.Abs(vec.x), Math.Abs(vec.y));

        /// <summary>
        /// returns on which quadrant of the vertex i of polygon vertexToTest is:
        ///   ^    <
        ///    \-2/
        ///     \/
        ///   0 /\ 0
        ///    / 2\
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="i"></param>
        /// <param name="vertexToTest"></param>
        /// <returns></returns>
        public static int WhichSideAt(this Polygon2d polygon, int i, Vector2d vertexToTest, double tol = 0)
        {
            (var pc, var cn) = polygon.SegmentsAt(i);
            var side1 = pc.WhichSide(vertexToTest, tol);
            var side2 = cn.WhichSide(vertexToTest, tol);
            return side1 + side2;
        }

        public static bool ContainsOrIntersects(this Polygon2d polygon, Polygon2d polygon2, double tolerance = MathUtil.ZeroTolerance)
        {
            if (!polygon.Bounds.Intersects(polygon2.Bounds))
            {
                return false;
            }

            if (polygon.ContainsInclusive(polygon2[0], tolerance)) return true;
            if (polygon2.ContainsInclusive(polygon[0], tolerance)) return true;
            return AnyEdgesIntersect(polygon, polygon2, tolerance);
        }

        public static bool Intersects(this Polygon2d polygon, Polygon2d polygon2, double tolerance = MathUtil.ZeroTolerance)
        {
            if (!polygon.Bounds.Intersects(polygon2.Bounds))
            {
                return false;
            }

            return AnyEdgesIntersect(polygon, polygon2, tolerance);
        }

        private static bool AnyEdgesIntersect(Polygon2d polygon, Polygon2d polygon2, double tolerance)
        {
            Segment2d seg1;
            Segment2d seg2;
            var verts1 = polygon.VerticesAsReadOnlySpan;
            var verts2 = polygon2.VerticesAsReadOnlySpan;
            Vector2d prev1 = verts1[^1];
            Vector2d prev2 = verts2[^1];
            for (int i = 0; i < verts1.Length; i++)
            {
                Vector2d vert1 = verts1[i];
                seg1 = new Segment2d(prev1, vert1);
                for (int j = 0; j < verts2.Length; j++)
                {
                    Vector2d vert2 = verts2[j];
                    seg2 = new Segment2d(prev2, vert2);
                    if (seg1.Intersects(seg2, tolerance))
                    {
                        return true;
                    }
                    prev2 = vert2;
                }
                prev1 = vert1;
            }

            return false;
        }

        public static bool IntersectsNotParallel(this in Segment2d seg1, in Segment2d seg2, double dotThresh = MathUtil.ZeroTolerance, double intervalThresh = 0)
            => IntersectsNotParallel(seg1, seg2, out _, out _, dotThresh, intervalThresh);

        /// <summary>
        /// Tests if two segments intersect, ignoring parallel overlaps.
        /// If they intersect, returns the interpolation distances t1 and t2 (relative to segment centers).
        /// </summary>
        public static bool IntersectsNotParallel(this in Segment2d seg1, in Segment2d seg2, out double t1, out double t2, double dotThresh = MathUtil.ZeroTolerance, double intervalThresh = 0)
        {
            double D0DotPerpD1 = seg1.Direction.DotPerp(seg2.Direction);
            if (Math.Abs(D0DotPerpD1) <= dotThresh)
            {
                // Lines are parallel, IntersectsNotParallel check will return false in that case
                t1 = seg1.Project(seg2.Center);
                t2 = seg2.Project(seg1.Center);
                return false;
            }

            // Lines intersect in a single point.
            Vector2d diff = seg2.Center - seg1.Center;
            double diffDotPerpD0 = diff.DotPerp(seg1.Direction);
            double diffDotPerpD1 = diff.DotPerp(seg2.Direction);
            double invD0DotPerpD1 = 1.0 / D0DotPerpD1;
            t1 = diffDotPerpD1 * invD0DotPerpD1;
            t2 = diffDotPerpD0 * invD0DotPerpD1;

            // Only report an intersection if t1 and t2 fall within the extents (with tolerance).
            return Math.Abs(t1) <= (seg1.Extent + intervalThresh)
                && Math.Abs(t2) <= (seg2.Extent + intervalThresh);
        }

        public static bool Intersects(this in Segment2d seg1, in Segment2d seg2, out double t1, out double t2, double dotThresh = MathUtil.ZeroTolerance, double intervalThresh = 0)
        {
            Vector2d diff = seg2.Center - seg1.Center;
            double D0DotPerpD1 = seg1.Direction.DotPerp(seg2.Direction);
            if (Math.Abs(D0DotPerpD1) > dotThresh)
            {   // Lines intersect in a single point.
                double invD0DotPerpD1 = 1 / D0DotPerpD1;
                double diffDotPerpD0 = diff.DotPerp(seg1.Direction);
                double diffDotPerpD1 = diff.DotPerp(seg2.Direction);
                t1 = diffDotPerpD1 * invD0DotPerpD1;
                t2 = diffDotPerpD0 * invD0DotPerpD1;
                return Math.Abs(t1) <= (seg1.Extent + intervalThresh)
                        && Math.Abs(t2) <= (seg2.Extent + intervalThresh);
            }

            // Lines are parallel.
            diff.Normalize();
            double diffNDotPerpD1 = diff.DotPerp(seg2.Direction);
            if (Math.Abs(diffNDotPerpD1) <= dotThresh)
            {
                // Compute the location of segment1 endpoints relative to segment0.
                diff = seg2.Center - seg1.Center;
                double tseg1 = seg1.Direction.Dot(diff);
                t1 = -tseg1 + seg1.Extent;
                t2 = tseg1 - seg2.Extent;
                double tmax = tseg1 + seg2.Extent;
                if (!(t2 > seg1.Extent))
                {
                    return !(tmax < -seg1.Extent);
                }
                return true;
            }

            // lines are parallel but not collinear
            t1 = double.NaN;
            t2 = double.NaN;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d NextVertexFrom(this ReadOnlySpan<Vector2d> vertices, int i)
        {
            int wrappedI = i + 1;
            //benchmarked: ternary op is 5 times faster than modulo
            wrappedI = wrappedI < vertices.Length ? wrappedI : 0;
            return vertices[wrappedI];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d PrevVertexFrom(this ReadOnlySpan<Vector2d> vertices, int i)
        {
            int wrappedI = i - 1;
            //benchmarked: ternary op is 5 times faster than modulo
            wrappedI = wrappedI < 0 ? vertices.Length - 1 : wrappedI;
            return vertices[wrappedI];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d GetVertexWrapped(this Polygon2d polygon, int i)
        {
            int wrappedI = i % polygon.VertexCount;
            if (wrappedI < 0) wrappedI += polygon.VertexCount;
            return polygon.Vertices[wrappedI];
        }

        public static Vector2d EdgeNormal(this Polygon2d polygon, int i)
        {
            var verts = polygon.VerticesAsReadOnlySpan;
            Vector2d start = verts[i];
            Vector2d end = verts.NextVertexFrom(i);

            return (end - start).Perp.Normalized;
        }

        /// <summary>
        /// Divide all edges that are longer than maxEdgeLength into subsections with length < maxEdgeLength.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="maxEdgeLength"></param>
        /// <returns></returns>
        public static Polygon2d SectionToMaxEdgeLength(this Polygon2d polygon, double maxEdgeLength)
        {
            var sectionedPoly = new Polygon2d();
            foreach (var segment in polygon.SegmentItr())
            {
                var length = segment.Length;
                if (length > maxEdgeLength)
                {
                    double lengthAdded = 0;
                    int i = 0;
                    while (lengthAdded < length - maxEdgeLength)
                    {
                        i++;
                        sectionedPoly.AppendVertex(segment.SampleArcLength(i * maxEdgeLength));
                        lengthAdded += maxEdgeLength;
                    }
                }
                sectionedPoly.AppendVertex(segment.P1);
            }
            return sectionedPoly;
        }

        public static (double min, double max) MinMaxEdgeLength(this IEnumerable<Polygon2d> polygons)
        {
            double min = double.MaxValue;
            double max = double.MinValue;
            foreach (var poly in polygons)
            {
                Vector2d prev = poly.Vertices[^1];
                foreach (var vert in poly.Vertices)
                {
                    var edgeLength = (vert - prev).LengthSquared;
                    min = Math.Min(min, edgeLength);
                    max = Math.Max(max, edgeLength);
                    prev = vert;
                }
            }
            //no edges?
            if (min == double.MaxValue) { return (0, 0); }
            return (Math.Sqrt(min), Math.Sqrt(max));
        }

        /// <summary>
        /// <author>MFroehlich</author>
        /// Collapse all edges with angles lower than minAngle, 
        /// if the edge created by collapsing is shorter than tolerance.
        /// intended for removing clipping artifacts.
        /// </summary>
        /// <param name=""></param>
        /// <param name="angle"></param> minimum angle at vertex in degrees
        public static bool CollapseSpikes(this Polygon2d poly, double tolerance, double minAngle = 0.1)
            => HandleSpikes(poly, tolerance, minAngle, RemoveVertex);
        public static bool SpikesToRectangle(this Polygon2d poly, double tolerance, double minAngle = 0.1)
            => HandleSpikes(poly, tolerance, minAngle, SpikeToRectangle);

        private static void RemoveVertex(Polygon2d poly, int i, double _) => poly.RemoveVertex(i);
        private static void SpikeToRectangle(Polygon2d poly, int i, double tolerance)
        {
            var vtx = poly[i];
            var dir = poly.GetNormal_FaceAvg(i).UnitPerp;
            var newVtx = vtx + dir * tolerance;
            poly.InsertVertex(i, newVtx);
        }

        private static bool HandleSpikes(Polygon2d poly, double tolerance, double minAngle, Action<Polygon2d, int, double> spikeHandler)
        {
            bool handledSomething = false;
            double tolSqr = tolerance * tolerance;
            var vertices = poly.VerticesAsReadOnlySpan;
            //iterate loop reversed because the handlers insert/remove vertices at i => lower i still valid
            for (int i = vertices.Length - 1; i >= 0; i--)
            {
                if (poly.AngleDAtVertex(i) < minAngle)
                {
                    var prev = vertices.PrevVertexFrom(i);
                    var next = vertices.NextVertexFrom(i);
                    var distSqr = (prev - next).LengthSquared;
                    if (distSqr < tolSqr)
                    {
                        spikeHandler(poly, i, tolerance);
                        handledSomething = true;
                    }
                }
            }
            return handledSomething;
        }

        /// <summary>
        /// Return the outer contour together with the holes in a single IEnumerable
        /// </summary>
        /// <param name="genPoly"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> OuterAndHolesItr(this GeneralPolygon2d genPoly)
        {
            yield return genPoly.Outer;
            foreach (var hole in genPoly.Holes)
                yield return hole;
        }

        public static IEnumerable<Polygon2d> OuterAndHolesItr(this IEnumerable<GeneralPolygon2d> genPolys)
        {
            foreach (var genPoly in genPolys)
                foreach (var poly in genPoly.OuterAndHolesItr())
                    yield return poly;
        }

        public static Polygon2d OuterOrHoleByIdx(this GeneralPolygon2d genPoly, int idx)
        {
            if (idx < 0)
                return genPoly.Outer;
            else
                return genPoly.Holes[idx];
        }

        public static void RemoveCoincidentVertices(this GeneralPolygon2d genPoly, double tolerance = MathUtil.ZeroTolerance)
        {
            foreach (var poly in genPoly.OuterAndHolesItr())
            {
                // MFRoehlich: cast is a potential problem
                poly.RemoveConsecutiveCoincidentVertices(tolerance);
            }
        }

        public static bool AreVerticesCoincident(this Polygon2d polygon, int i, int j, double toleranceSqr = MathUtil.ZeroTolerance)
        {
            return (polygon.GetVertexWrapped(i) - polygon.GetVertexWrapped(j)).LengthSquared < toleranceSqr;
        }

        public static void MoveSelfTouchingVertices(this GeneralPolygon2d polygon, double moveDistance = 0.0001)
        {
            var vertexCount = polygon.VertexCount;
            var vertexSet = new HashSet<Vector2d>(vertexCount);
            polygon.Outer.MoveSelfTouchingVertices(vertexSet, moveDistance);
            foreach (var hole in polygon.Holes) hole.MoveSelfTouchingVertices(vertexSet, moveDistance);
        }

        public static Polygon2d Shadow(this Box3d box)
        {
            // corners [ (-x,-y), (x,-y), (x,y), (-x,y) ], -z, then +z
            //
            //   7---6     +z       or        3---2     -z
            //   |\  |\                       |\  |\
            //   4-\-5 \                      0-\-1 \
            //    \ 3---2                      \ 7---6   
            //     \|   |                       \|   |
            //      0---1  -z                    4---5  +z
            //
            var vertices = new Vector2d[4];
            vertices[0] = box.Corner(3).To2d();
            vertices[1] = box.Corner(2).To2d();
            vertices[2] = box.Corner(1).To2d();
            vertices[3] = box.Corner(0).To2d();
            var shadow = new Polygon2d(vertices);
            if (shadow.IsHole)
                shadow.Reverse();
            return shadow;
        }

        /// <summary>
        /// Calculate intersection point between line and segment.
        /// Returns Vector2d.MaxValue if lines are parallel or line does not intersect segment.
        /// </summary>
        /// <returns></returns>
        public static Vector2d IntersectionPoint(this Line2d line, Segment2d segment, double dotThresh = MathUtil.ZeroTolerance, double intervalThresh = 0)
        {
            // see IntrLine2Line2 for explanation of algorithm
            Vector2d diff = segment.Center - line.Origin;
            double D0DotPerpD1 = line.Direction.DotPerp(segment.Direction);
            if (Math.Abs(D0DotPerpD1) > dotThresh)
            {                    // Lines intersect in a single point.
                double invD0DotPerpD1 = 1 / D0DotPerpD1;
                double diffDotPerpD1 = diff.DotPerp(line.Direction);
                double s = diffDotPerpD1 * invD0DotPerpD1;
                if (Math.Abs(s) > (segment.Extent + intervalThresh))
                    return Vector2d.MaxValue;
                else
                    return segment.Center + s * segment.Direction;
            }
            // Lines are parallel.
            return Vector2d.MaxValue;
        }

        public static AxisAlignedBox2d Bounds(this IEnumerable<Vector2d> points)
        {
            var bounds = AxisAlignedBox2d.Empty;
            foreach (var point in points) { bounds.Contain(point); }
            return bounds;
        }

        public static AxisAlignedBox2d Bounds(this IEnumerable<Polygon2d> polygons)
        {
            using var enumerator = polygons.GetEnumerator();
            if (!enumerator.MoveNext())
                return AxisAlignedBox2d.Empty;

            AxisAlignedBox2d bounds = enumerator.Current.Bounds;
            while(enumerator.MoveNext())
            {
                bounds.Contain(enumerator.Current.Bounds);
            }
            return bounds;
        }

        public static AxisAlignedBox2d Bounds(this IEnumerable<GeneralPolygon2d> genPolygons) => genPolygons.Select(x => x.Outer).Bounds();

        public record struct ExtremePointData
        {
            public int iMin;
            public int iMax;
            public Polygon2d convexHull;
            public Box2d minimalBoundingBox;
        }

        public static ExtremePointData FindExtremePoints(this Polygon2d polygon)
        {
            var result = new ExtremePointData();
            result.convexHull = polygon.ConvexHull();
            result.minimalBoundingBox = new ContMinBox2(result.convexHull.Vertices, 0, QueryNumberType.QT_DOUBLE, isConvexPolygon: true).MinBox;

            var shortAxis = result.minimalBoundingBox.MinExtendAxis();
            var longAxis = result.minimalBoundingBox.MaxExtendAxis();

            double min = double.MaxValue;
            double max = double.MinValue;
            for (int i = 0; i < polygon.VertexCount; i++)
            {
                var vtx = polygon[i];
                var signedDist = shortAxis.SignedDistanceSqrd(vtx);
                //if we have an edge that is parallel to the coordinate system
                //decide by distance to other axis
                if (signedDist < min)
                {
                    min = signedDist;
                    result.iMin = i;
                }
                else if (signedDist > max)
                {
                    max = signedDist;
                    result.iMax = i;
                }
            }
            return result;
        }

        public static double SignedDistanceSqrd(this Line2d line, Vector2d vec) => line.DistanceSquared(vec) * line.WhichSide(vec);

        public static double ClosestVertexDistanceSquared(this Polygon2d polygon, Vector2d p, out int closestIdx)
        {
            double distance = double.MaxValue;
            closestIdx = -1;
            for (int i = 0; i < polygon.Vertices.Count; i++)
            {
                var vtxDist = polygon[i].DistanceSquared(p);
                if (vtxDist < distance)
                {
                    distance = vtxDist;
                    closestIdx = i;
                }
            }
            return distance;
        }

        public static bool DistanceSquaredGreaterThan(this GeneralPolygon2d polygon, Vector2d p, double distMax, out double distance, out int iHoleIndex, out int iNearSeg, out double fNearSegT)
        {
            iNearSeg = iHoleIndex = -1;
            fNearSegT = double.MaxValue;
            bool greater = polygon.Outer.DistanceSquaredGreaterThan(p, distMax, out distance, out iNearSeg, out fNearSegT);
            if (!greater) return false;
            var holes = polygon.Holes;
            for (int i = 0; i < holes.Count; ++i)
            {
                int seg; double segt;
                greater = holes[i].DistanceSquaredGreaterThan(p, distMax, out double holedist, out seg, out segt);
                if (!greater) return false;
                if (holedist < distance)
                {
                    distance = holedist;
                    iHoleIndex = i;
                    iNearSeg = seg;
                    fNearSegT = segt;
                }
            }
            return true;
        }

        public static bool DistanceSquaredGreaterThan(this Polygon2d polygon, Vector2d p, double distMax, out double distance, out int iNearSeg, out double fNearSegT)
        {
            iNearSeg = -1;
            fNearSegT = double.MaxValue;
            distance = double.MaxValue;
            int N = polygon.VertexCount;
            var vertices = polygon.VerticesAsReadOnlySpan;
            for (int vi = 0; vi < N; ++vi)
            {
                int next = vi == N - 1 ? 0 : vi + 1;
                Segment2d seg = new Segment2d(vertices[vi], vertices[next]);
                double t = (p - seg.Center).Dot(seg.Direction);
                double d;
                if (t >= seg.Extent)
                    d = seg.P1.DistanceSquared(p);
                else if (t <= -seg.Extent)
                    d = seg.P0.DistanceSquared(p);
                else
                    d = (seg.PointAt(t) - p).LengthSquared;

                //early out check
                if (d < distMax) return false;
                if (d < distance)
                {
                    distance = d;
                    iNearSeg = vi;
                    fNearSegT = t;
                }
            }
            return true;
        }

        /// <summary>
        /// Find the minimum distance between two polygons, assuming they do not intersect.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public static double DistanceTo(this Polygon2d p1, Polygon2d p2)
        {
            return ConvexPolygonOps.NaiveDistance(p1, p2);
        }

        public static double DistanceTo(this IEnumerable<Polygon2d> polys, Polygon2d p2)
        {
            return polys.Min(p2.DistanceTo);
        }


        /// <summary>
        /// Finds the nearest point of a polygon from p by iterating through the polygons segments.
        /// If many queries are done, Polygon2dBoxTree should be used instead.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        public static Vector2d NearestPoint(this IEnumerable<Polygon2d> polygons, Vector2d p, double tolerance = MathUtil.ZeroTolerance) => NearestPoint(polygons, p, out _, tolerance);

        /// <summary>
        /// Finds the nearest point of a polygon from p by iterating through the polygons segments.
        /// If many queries are done, Polygon2dBoxTree should be used instead.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="p"></param>
        /// <returns></returns>
        public static Vector2d NearestPoint(this IEnumerable<Polygon2d> polygons, Vector2d p, out int polyIdx, double tolerance = MathUtil.ZeroTolerance)
        {
            var minDist = double.MaxValue;
            Vector2d result = p;
            int idx = 0;
            polyIdx = -1;
            foreach (var polygon in polygons)
            {
                double dist = polygon.DistanceSquared(p, out _, out _, out Vector2d closestPoint, tolerance);
                if (dist < minDist)
                {
                    result = closestPoint;
                    minDist = dist;
                    polyIdx = idx;
                }
                idx++;
            }

            return result;
        }

        public static double DistanceTo(this IEnumerable<Polygon2d> polygons, Vector2d point)
        {
            return Math.Sqrt(polygons.Min(p => p.DistanceSquared(point)));
        }

        /// <summary>
        /// Filters the general polygons and their holes by a minimum area.
        /// All polygons smaller than minArea and holes smaller than minarea will be discarded.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="minArea"></param>
        /// <returns></returns>
        public static IEnumerable<GeneralPolygon2d> FilterMinArea(this IEnumerable<GeneralPolygon2d> polygons, double minArea = MathUtil.ZeroTolerance)
        {
            foreach (var poly in polygons)
            {
                if (poly.Outer.Area > minArea)
                {
                    var filteredShadow = new GeneralPolygon2d(poly.Outer);
                    foreach (var hole in poly.Holes)
                    {
                        if (hole.Area > minArea)
                            filteredShadow.AddHole(hole, false, false);
                    }
                    yield return filteredShadow;
                }
            }
        }


        public static IEnumerable<Polygon2d> FilterByMinArea(this IEnumerable<Polygon2d> polygons, double minArea = MathUtil.ZeroTolerance)
        {
            foreach (var poly in polygons)
            {
                if (poly.Area > minArea)
                {
                    yield return poly;
                }
            }
        }


        /// <summary>
        /// Filters the polygons by a minimum area.
        /// All polygons smaller than minArea will be discarded.
        /// </summary>
        /// <param name="polygons"></param>
        /// <param name="minArea"></param>
        /// <returns></returns>
        public static IEnumerable<Polygon2d> FilterByMinAreaReturnToPool(this IEnumerable<Polygon2d> polygons, double minArea = MathUtil.ZeroTolerance)
        {
            foreach (var poly in polygons)
            {
                if (poly.Area > minArea)
                {
                    yield return poly;
                }
                else SharedPolyPool.Return(poly);
            }
        }

        public static Polygon2d CloneAndSimplify(this Polygon2d polygon, double minEdgeLength = MathUtil.Epsilonf, double lineDeviationTolerance = 0)
        {
            var copy = polygon.Duplicate();
            copy.Simplify(minEdgeLength, lineDeviationTolerance, true);
            return copy;
        }

        public static List<Polygon2d> CloneAndSimplify(this IEnumerable<Polygon2d> polygons, double minEdgeLength = MathUtil.Epsilonf, double lineDeviationTolerance = 0)
        {
            var output = new List<Polygon2d>();
            foreach (var poly in polygons)
            {
                output.Add(poly.CloneAndSimplify(minEdgeLength, lineDeviationTolerance));
            }
            return output;
        }

        public static void SimplifyShifted(this GeneralPolygon2d genPoly, double clusterTol = 0.0001, double lineDeviationTol = 0.01, bool bSimplifyStraightLines = true)
        {
            genPoly.Outer.Simplify(clusterTol, lineDeviationTol, bSimplifyStraightLines, shiftStart: true);
            foreach (var hole in genPoly.Holes)
            {
                hole.Simplify(clusterTol, lineDeviationTol, bSimplifyStraightLines, shiftStart: true);
            }
        }

        public static void SimplifyShifted(this IEnumerable<GeneralPolygon2d> genPolys, double clusterTol = 0.0001, double lineDeviationTol = 0.01, bool bSimplifyStraightLines = true)
        {
            foreach (var polygon in genPolys) { polygon.SimplifyShifted(clusterTol, lineDeviationTol, bSimplifyStraightLines); }
        }

        public static bool ContainsOrOnLine(this g3.GeneralPolygon2d genPoly, Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            if (!genPoly.Outer.ContainsInclusive(P, epsilon))
                return false;

            //for holes we use strict contains 
            foreach (var hole in genPoly.Holes)
                if (hole.Contains(P))
                    return false;

            return true;
        }

        public static SplitPolygons DeleteEdgesOnLine(this IEnumerable<Polygon2d> polygons, Line2d line)
        {
            var result = new SplitPolygons();

            foreach (var polygon in polygons)
            {
                var firstSide = line.WhichSide(polygon.Vertices[0]);
                var side = firstSide;
                var polyline = new PolyLine2d();
                int sideSwitches = 0;

                for (int i = 0; i < polygon.Vertices.Count; i++)
                {
                    Vector2d vert = polygon.Vertices[i];
                    var curSide = line.WhichSide(vert);
                    if (curSide == side)
                    {
                        polyline.AppendVertex(vert);
                    }
                    else
                    {
                        sideSwitches++;
                        result.AddPolyline(polyline, side);
                        polyline = new PolyLine2d();
                        side = curSide;
                    }
                }

                if (sideSwitches == 0)
                {
                    polyline.AppendVertex(polyline.Vertices[0]);
                    result.AddPolyline(polyline, side);
                }
                else if (sideSwitches % 2 == 1)
                {
                    result.AddPolyline(polyline, side);
                }
                else
                {
                    //we ended back on the same side again, merge with first polyline
                    if (side == 1)
                    {
                        polyline.AppendVertices(result.positiveSide[0]);
                        result.positiveSide[0] = polyline;
                    }
                    else
                    {
                        polyline.AppendVertices(result.negativeSide[0]);
                        result.negativeSide[0] = polyline;
                    }
                }
            }
            return result;
        }

        /// <summary>
        /// Extrudes a stack of general polygons (potentially with holes) into a 3d mesh.
        /// The input slices are sorted by zHeigth, each slice is extruded upwards (in positive z direction) up to the next slice.
        /// The end of the highest extrusion must be passed additionally via zHeightEnd, the cap contour is implicitly specified by the last slice.
        /// </summary>
        /// <param name="input">the stack of polygons to extrude</param>
        /// <param name="zHeightEnd">end of the extrusion, must be greater than all zHeights of the input</param>
        /// <returns></returns>
        /// <exception cref="ArgumentException">thrown if zHeights would create intersecting or zero-volume geometry</exception>
        public static DMesh3 ExtrudedPolygonStack(IEnumerable<(double zHeight, List<GeneralPolygon2d> genPolys)> input, double zHeightEnd)
        {
            var slices = input.ToArray();
            if(slices.Length == 0)
                return new DMesh3(MeshComponents.None);

            slices.SortBy(x => x.zHeight);
            if (zHeightEnd <= slices[^1].zHeight)
                throw new ArgumentException("zHeightEnd must be larger than largest input zHeight", nameof(zHeightEnd));
            MeshEditor meshEditor = new MeshEditor(new DMesh3(MeshComponents.None));

            // special handling for first slice: only append cap
            (double previousHeight, List<GeneralPolygon2d> previousSlice) = slices[0];
            AppendCapMesh(null, previousSlice, previousHeight);

            // extrude polygons and create caps in-between
            for (int i = 1; i < slices.Length; i++)
            {
                (double height, List<GeneralPolygon2d> currentSlice) = slices[i];
                AppendTubeMesh(previousSlice, previousHeight, height);
                AppendCapMesh(previousSlice, currentSlice, height);
                previousSlice = currentSlice;
                previousHeight = height;
            }

            // special handling for last slice: extrude last tubes, append last cap again without XOR
            AppendTubeMesh(previousSlice, previousHeight, zHeightEnd);
            AppendCapMesh(previousSlice, null, zHeightEnd);

            // weld the mesh parts together TODO: result is not yet satisfying but visually kinda ok
            // main problem is that the orientation of XOR cap is always up, so its wrong for downskin surfaces
            var resultMesh = meshEditor.Mesh;
            var repair = new MergeCoincidentEdges(resultMesh);
            repair.MergeDistance = 0.1;
            repair.Apply();

            return resultMesh;

            void AppendTubeMesh(List<GeneralPolygon2d> slice, double startHeight, double endHeight)
            {
                double thickness = endHeight - startHeight;
                if(thickness <= 0)
                    throw new ArgumentException("input contains duplicate zHeights (creates zero-volume geometry)", nameof(input));
                var polyContourMesh = slice.Extrude(thickness);
                MeshTransforms.Translate(polyContourMesh, startHeight * new Vector3d(0, 0, 1));
                meshEditor.AppendMesh(polyContourMesh);
            }

            void AppendCapMesh(List<GeneralPolygon2d> bottomSlice, List<GeneralPolygon2d> topSlice, double height)
            {
                // generate cap
                var capMesh = bottomSlice.XOrAndTriangulate(topSlice);
                MeshTransforms.Translate(capMesh, height * new Vector3d(0, 0, 1));
                meshEditor.AppendMesh(capMesh);
            }
        }


        /// <summary>
        /// Calculate the XOR of two Lists of GeneralPolygon2ds and triangulate the result.
        /// </summary>
        /// <param name="bottomSlice"></param>
        /// <param name="topSlice"></param>
        /// <returns></returns>
        public static DMesh3 XOrAndTriangulate(this List<GeneralPolygon2d> bottomSlice, List<GeneralPolygon2d> topSlice)
        {
            if (bottomSlice is null & topSlice is null)
                throw new ArgumentNullException(nameof(bottomSlice), "both slices null");

            List<GeneralPolygon2d> upskin = topSlice switch
            {
                null => bottomSlice,
                _ => bottomSlice.ClipBy(topSlice),
            };

            List<GeneralPolygon2d> downskin = bottomSlice switch
            {
                null => topSlice,
                _ => topSlice.ClipBy(bottomSlice),
            };

            MeshEditor meshEditor = new MeshEditor(new DMesh3());
            AppendPolyDmesh(upskin, false);
            AppendPolyDmesh(downskin, true);
            return meshEditor.Mesh;

            void AppendPolyDmesh(List<GeneralPolygon2d> polygons, bool flip)
            {
                // remove small polygon artifacts
                polygons.RemoveAll(x => Math.Abs(x.Area) < 0.1);
                // offset to remove zero-area geometry
                var offset = polygons.Offset(1e-2);
                //append to meshEditor
                foreach (var genPoly in offset)
                {
                    var mesh = genPoly.Triangulate();
                    if (flip)
                        mesh.ReverseOrientation();
                    meshEditor.AppendMesh(mesh);
                }
            }
        }

        /// <summary>
        /// Extrude an IEnumerable of GeneralPolygon2s into tubes.
        /// All tubes are oriented in z-direction and returned as a single DMesh.
        /// </summary>
        /// <param name="genPolys"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        public static DMesh3 Extrude(this IEnumerable<GeneralPolygon2d> genPolys, double length)
        {
            MeshEditor meshEditor = new MeshEditor(new DMesh3(MeshComponents.None));
            var extrusionPath = new List<Vector3d> { Vector3d.Zero, length * Vector3d.AxisZ };
            foreach (var genPoly in genPolys)
            {
                foreach (var poly in genPoly.OuterAndHolesItr())
                {
                    var generator = new TubeGenerator(new DCurve3(extrusionPath, false), poly);
                    generator.Capped = false;
                    generator.Generate();
                    var mesh = generator.MakeDMesh();
                    meshEditor.AppendMesh(mesh);
                }
            }
            return meshEditor.Mesh;
        }

        public static DMesh3 Extrude(this Polygon2d polygon, double length)
        {
            MeshEditor meshEditor = new MeshEditor(new DMesh3(MeshComponents.None));
            var extrusionPath = new List<Vector3d> { Vector3d.Zero, length * Vector3d.AxisZ };
            var generator = new TubeGenerator(new DCurve3(extrusionPath, false), polygon);
            generator.Capped = false;
            generator.Generate();
            meshEditor.AppendMesh(generator.MakeDMesh());

            var bottomCap = polygon.Triangulate();
            bottomCap.ReverseOrientation();
            meshEditor.AppendMesh(bottomCap);

            var topCap = polygon.Triangulate();
            MeshTransforms.Translate(topCap, length * new Vector3d(0, 0, 1));
            meshEditor.AppendMesh(topCap);

            var resultMesh = meshEditor.Mesh;

            var mergeCoincidentEdges = new MergeCoincidentEdges(resultMesh);
            mergeCoincidentEdges.MergeDistance = 0.001;
            mergeCoincidentEdges.OnlyUniquePairs = true;
            mergeCoincidentEdges.Apply();

            return resultMesh;
        }

        /// <summary>
        /// Create a DMesh3 from an IEnumerable of GeneralPolygon2ds
        /// by triangulating the polygon.
        /// </summary>
        /// <param name="genPolys"></param>
        /// <returns></returns>
        public static DMesh3 Triangulate(this IEnumerable<GeneralPolygon2d> genPolys)
        {
            MeshEditor meshEditor = new MeshEditor(new DMesh3());
            foreach (var genPoly in genPolys)
            {
                meshEditor.AppendMesh(genPoly.Triangulate());
            }
            return meshEditor.Mesh;
        }

        public static DMesh3 Triangulate(this Polygon2d polygon) => new GeneralPolygon2d(polygon).Triangulate();

        /// <summary>
        /// Create a DMesh3 from a single GeneralPolygon2d
        /// by triangulating the polygon.
        /// </summary>
        /// <param name="genPoly"></param>
        /// <param name="meshEditor"></param>
        public static DMesh3 Triangulate(this GeneralPolygon2d genPoly, Vector3f normal = default)
        {
            if (normal == default) normal = Vector3f.AxisZ;
            var generatorFill = new TriangulatedPolygonGenerator() { FixedNormal = normal };
            generatorFill.Polygon = genPoly;
            generatorFill.Generate();
            var mesh = generatorFill.MakeDMesh();
            return mesh;
        }

        public static bool Contains(this IEnumerable<GeneralPolygon2d> genPolys, Vector2d vector)
        {
            bool contains = false;
            foreach (var genPoly in genPolys)
            {
                if (genPoly.Contains(vector))
                {
                    contains = true;
                    break;
                }
            }
            return contains;
        }

        public static Circle2d NoFitCircle(this Circle2d fixedCircle, Circle2d movingCircle)
        {
            return new Circle2d(fixedCircle.Center - movingCircle.Center, fixedCircle.Radius + movingCircle.Radius);
        }
    }
}
