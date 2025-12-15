using Clipper2Lib;
using g3;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace g3
{
    /// <summary>
    /// Implementation of the slicing algorithm proposed in http://dx.doi.org/10.1007/s00170-015-7071-9
    /// Does one iteration over all triangles and slices each triangle multiple times if necessary.
    /// For each triangle, one intersection point between one edge and the slicing plane is calculated.
    /// Intersections are then sorted into contours using linked lists.
    /// Only works on closed meshes.
    /// </summary>
    public class ECCSlicer
    {
        /// <summary>
        /// Compute single slice at a given z-height.
        /// </summary>
        public static List<Polygon2d> SingleSlice(DMesh3 mesh, double z)
        {
#if DEBUG
            var debugViewPoints = new List<Vector2d>();
#endif
            var slice = new Slice();
            foreach (var triIdx in mesh.TriangleIndices())
            {
                // obtain triangle data
                var t = GetTriVertexData(mesh, mesh.GetTriangle(triIdx));

                // cut triangle
                if (z < t.vMin.coords.z || z > t.vMax.coords.z) continue;   // no intersection
                if (t.vMin.coords.z == t.vMax.coords.z) continue;           // ignore triangles parallel to plane

                var intersect = new IntersectStruct();
                // intersection in lower half (includes the edge case where s3 lies on the plane)
                if (z < t.vMed.coords.z || t.vMed.coords.z == t.vMax.coords.z)
                {
                    if (t.edgeOrder == EdgeOrder.S1_BACKWARD) { intersect.e1 = t.s2; intersect.e2 = t.s1; }
                    else { intersect.e1 = t.s1; intersect.e2 = t.s2; }
                }
                // intersection in upper half
                else
                {
                    if (t.edgeOrder == EdgeOrder.S1_BACKWARD) { intersect.e1 = t.s3; intersect.e2 = t.s1; }
                    else { intersect.e1 = t.s1; intersect.e2 = t.s3; }
                }
                // calculate intersection only for the forward edge
                intersect.vInter = ZLineIntersect(intersect.e1.start.coords, intersect.e1.end.coords, z);

                // insert into contour
                slice.Insert(intersect);
#if DEBUG
                debugViewPoints.Add(intersect.vInter.xy);
#endif
            }
            return slice.GetClosedLoops();
        }

        /// <summary>
        /// Compute all slices at once.
        /// The z value of each slice is zZero + slice number * sliceThickness.
        /// </summary>
        /// <returns>A dictionary containing (slice number, contour polygons)</returns>
        public static Dictionary<int, List<Polygon2d>> FullSlice(DMesh3 mesh, double zZero = 0, double sliceThickness = 1)
        {
            var slices = new Dictionary<int, Slice>();
            foreach (var triIdx in mesh.TriangleIndices())
            {
                // obtain vertex data
                var t = GetTriVertexData(mesh, mesh.GetTriangle(triIdx));

                // calculate slice numbers for each vertex
                int sMin = SliceNumber(t.vMin.coords.z, zZero, sliceThickness);
                int sMed = SliceNumber(t.vMed.coords.z, zZero, sliceThickness);
                int sMax = SliceNumber(t.vMax.coords.z, zZero, sliceThickness);

                // lower triangle half
                for (int j = sMin; j < sMed; j++)
                {
                    var intersect = new IntersectStruct();
                    if (t.edgeOrder == EdgeOrder.S1_BACKWARD) { intersect.e1 = t.s2; intersect.e2 = t.s1; }
                    else { intersect.e1 = t.s1; intersect.e2 = t.s2; }

                    double zSlice = zZero + (j + 1) * sliceThickness;
                    intersect.vInter = ZLineIntersect(intersect.e1.start.coords, intersect.e1.end.coords, zSlice);

                    if (!slices.TryGetValue(j + 1, out Slice slice)) { slice = new Slice(); slices.Add(j + 1, slice); }
                    slice.Insert(intersect);
                }

                // upper triangle half
                for (int k = sMed; k < sMax; k++)
                {
                    var intersect = new IntersectStruct();
                    if (t.edgeOrder == EdgeOrder.S1_BACKWARD) { intersect.e1 = t.s3; intersect.e2 = t.s1; }
                    else { intersect.e1 = t.s1; intersect.e2 = t.s3; }

                    double zSlice = zZero + (k + 1) * sliceThickness;
                    intersect.vInter = ZLineIntersect(intersect.e1.start.coords, intersect.e1.end.coords, zSlice);

                    if (!slices.TryGetValue(k + 1, out Slice slice)) { slice = new Slice(); slices.Add(k + 1, slice); }
                    slice.Insert(intersect);
                }
            }

            // collect contours for each slice and return
            var result = new Dictionary<int, List<Polygon2d>>();
            foreach (int key in slices.Keys)
            {
                result.Add(key, slices[key].GetClosedLoops());
            }
            return result;
        }

        #region data structures

        public enum EdgeOrder { S1_BACKWARD, S1_FORWARD }

        internal struct Vertex
        {
            public int idx;         // index in DMesh
            public Vector3d coords; // coordinates in space
            public int flag;        // index in triangle
            public override bool Equals(object? other)
            {
                if (other == null || other is not Vertex) return false;
                return this.idx == ((Vertex)other).idx;
            }
            public override int GetHashCode() { return idx; }
            public static bool operator ==(Vertex a, Vertex b) { return a.Equals(b); }
            public static bool operator !=(Vertex a, Vertex b) { return !a.Equals(b); }
        }

        internal class Edge
        {
            public Vertex start;
            public Vertex end;
            public override bool Equals(object? other)
            {
                if (other == null || other is not Edge) return false;
                if (ReferenceEquals(this, other)) return true;
                else return (this.start == ((Edge)other).start && this.end == ((Edge)other).end)
                         || (this.start == ((Edge)other).end && this.end == ((Edge)other).start);
            }
            public override int GetHashCode()
            {
                return System.HashCode.Combine(start.GetHashCode, end.GetHashCode);
            }
            public static bool operator ==(Edge a, Edge b) { return a.Equals(b); }
            public static bool operator !=(Edge a, Edge b) { return !a.Equals(b); }
        }

        /// <summary>
        /// Denotes an intersection of a single triangle with a slicing plane.
        /// Only the intersection point on one edge ("forward edge") is stored,
        /// as well as references to both edges and their order.
        /// </summary>
        internal struct IntersectStruct
        {
            public Vector3d vInter;    // intersection point
            public Edge e1;            // forward edge
            public Edge e2;            // backward edge
        }

        /// <summary>
        /// Vertices are sorted by z. Edges are directed from lower to upper vertex.
        /// s1 always the longest (in z direction) edge. s2 and s3 are opposite, with
        /// s2 being the lower edge. Edge order indicates whether s1 is on the left
        /// or right side (when viewed from the outside, i.e. normal vector pointing
        /// at viewer). See paper for details.
        /// </summary>
        internal struct Triangle
        {
            public Vertex vMin, vMed, vMax;
            public Edge s1, s2, s3;
            public EdgeOrder edgeOrder;
        }

        #endregion

        #region helper functions
        /// <summary>
        /// Retrieve all relevant information on a specific triangle from the mesh.
        /// </summary>
        private static Triangle GetTriVertexData(DMesh3 mesh, Index3i triangle)
        {
            var verts = new Vertex[3];
            for (int i = 0; i < 3; i++)
            {
                verts[i] = new Vertex
                {
                    idx = triangle.array[i],
                    coords = mesh.GetVertex(triangle.array[i]),
                    flag = i
                };
            }
            verts = verts.SortBy(vert => vert.coords.z);
            return new Triangle()
            {
                vMin = verts[0],
                vMed = verts[1],
                vMax = verts[2],
                s1 = new() { start = verts[0], end = verts[2] },
                s2 = new() { start = verts[0], end = verts[1] },
                s3 = new() { start = verts[1], end = verts[2] },
                edgeOrder = DetermineEdgeOrder(verts[0], verts[1], verts[2]),
            };
        }

        /// <summary>
        /// Determine edge order for a triangle.
        /// Orientation of a triangle is given by order of vertices
        /// (counter-clockwise).
        /// </summary>
        private static EdgeOrder DetermineEdgeOrder(Vertex vMin, Vertex vMed, Vertex vMax)
        {
            int idxDiff = Math.Abs(vMin.flag - vMax.flag);
            if ((idxDiff == 1 && vMin.flag < vMax.flag) || (idxDiff == 2 && vMin.flag > vMax.flag))
                return EdgeOrder.S1_BACKWARD;
            //if ((idxDiff == 1 && vMin.flag > vMax.flag) || (idxDiff == 2 && vMin.flag < vMax.flag))
            else
                return EdgeOrder.S1_FORWARD;
        }

        /// <summary>
        /// Calculate the intersection point between a line segment and a xy-parallel plane at
        /// height z.
        /// </summary>
        /// <exception cref="ArgumentException"></exception>
        private static Vector3d ZLineIntersect(Vector3d lower, Vector3d upper, double z)
        {
#if DEBUG
            if (lower.z == upper.z) throw new ArgumentException("start and end z must be different");
            if (z < lower.z || z > upper.z) throw new ArgumentException("z not contained in line segment");
#endif
            double t = (z - lower.z) / (upper.z - lower.z);
            return lower + t * (upper - lower);
        }

        private static int SliceNumber(double z, double zZero, double sliceThickness)
        {
            // round down even for negative numbers
            return (int)Math.Floor((z - zZero) / sliceThickness);
        }

        #endregion

        #region contour construction

        private class Slice
        {
            /// <summary>Contour Linked List: holds all intersection linked lists in active construction.</summary>
            readonly List<LinkedList<IntersectStruct>> CLL = new();
            /// <summary>contains all intersection linked lists that have been closed.</summary>
            readonly List<LinkedList<IntersectStruct>> loops = new();

            /// <summary>
            /// Contour Construction algorithm.
            /// Try to find forward and backward matches in every intersection linked list,
            /// then insert the intersection struct and merge lists if necessary.
            /// (Not identical to the ECC proposed in the paper)
            /// </summary>
            public void Insert(IntersectStruct IS)
            {
                // check for forward and backward matches in all intersection linked lists (ILL)
                int forwardMatchPos = -1, backwardMatchPos = -1;
                for (int i = 0; i < CLL.Count; i++)
                {
                    if (IsForwardIntersection(CLL[i], IS)) { forwardMatchPos = i; break; }
                }
                for (int i = 0; i < CLL.Count; i++)
                {
                    if (IsBackwardIntersection(CLL[i], IS)) { backwardMatchPos = i; break; }
                }

                // no match anywhere, insert into new ILL
                if (forwardMatchPos == -1 && backwardMatchPos == -1)
                {
                    var ILL = new LinkedList<IntersectStruct>();
                    ILL.AddFirst(IS);
                    CLL.Add(ILL);
                    return;
                }
                // one forward match -> insert into matching ILL
                if (backwardMatchPos == -1)
                {
                    CLL[forwardMatchPos].AddFirst(IS);
                    return;
                }
                // one backward match -> insert into matching ILL
                if (forwardMatchPos == -1)
                {
                    CLL[backwardMatchPos].AddLast(IS);
                    return;
                }
                // both matches in same ILL -> found a closed loop
                if (forwardMatchPos == backwardMatchPos)
                {
                    CLL[forwardMatchPos].AddLast(IS);
                    loops.Add(CLL[forwardMatchPos]);
                    CLL.RemoveAt(forwardMatchPos);
                    return;
                }
                // both matches in different ILLs -> connect both ILLs
                else
                {
                    CLL[backwardMatchPos].AddLast(IS);
                    // apparently linked lists cannot be trivially concatenated
                    // in C#, maybe rethink using them here
                    foreach (var node in CLL[forwardMatchPos])
                    {
                        CLL[backwardMatchPos].AddLast(node);
                    }
                    CLL.RemoveAt(forwardMatchPos);
                    return;
                }
            }

            /// <summary>
            /// Return all closed contours that have been found up to this point
            /// as polygons.
            /// </summary>
            /// <returns></returns>
            public List<Polygon2d> GetClosedLoops()
            {
                var result = new List<Polygon2d>();
                foreach (var ILL in loops)
                {
                    var polygon = new Polygon2d();
                    foreach (var IS in ILL)
                    {
                        polygon.AppendVertex(IS.vInter.xy);
                    }
                    result.Add(polygon);
                }
                return result;
            }

            private static bool IsForwardIntersection(LinkedList<IntersectStruct> ILL, IntersectStruct IS)
            {
                if (ILL.First == null) return false;
                return IS.e2 == ILL.First.Value.e1;
            }

            private static bool IsBackwardIntersection(LinkedList<IntersectStruct> ILL, IntersectStruct IS)
            {
                if (ILL.Last == null) return false;
                return IS.e1 == ILL.Last.Value.e2;
            }
        }

        #endregion
    }

    /// <summary>
    /// Uses g3sharp's MeshPlaneCut to calculate a slice. For testing only.
    /// </summary>
    internal class MeshPlaneCutSlicer
    {
        internal static List<Polygon2d> SingleSlice(DMesh3 mesh, double z)
        {
            // MeshplaneCut changes the input mesh, so make a copy
            // (huge and unnecessary performance overhead)
            mesh = new DMesh3(mesh);
            var meshPlaneCut = new MeshPlaneCut(mesh, new Vector3d(0, 0, z), Vector3d.AxisZ);
            meshPlaneCut.Cut();

            var polygons = new List<Polygon2d>();
            foreach (var loop in meshPlaneCut.CutLoops)
            {
                var polygon = new Polygon2d();
                foreach (var vIdx in loop.Vertices)
                {
                    var vertex = mesh.GetVertex(vIdx);
                    polygon.AppendVertex(new Vector2d(vertex.x, vertex.y));
                }
                polygon.Simplify();
                polygon.Reverse();
                polygons.Add(polygon);
            }
            return polygons;
        }
    }
}
