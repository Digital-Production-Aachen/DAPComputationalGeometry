using g3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using static g3.ApproximateConvexDecomposition;

namespace g3
{
    // Port of Wm5ConvexHull2 from WildMagic5 library by David Eberly / geometrictools.com
    // ported and adapted another time from geometry3sharp to add an "on line" tolerance (using epsilon)

    // You have a choice of speed versus accuracy.  The fastest choice is
    // Query::QT_INT64, but it gives up a lot of precision, scaling the points
    // to [0,2^{20}]^3.  The choice Query::QT_INTEGER gives up less precision,
    // scaling the points to [0,2^{24}]^3.  The choice Query::QT_RATIONAL uses
    // exact arithmetic, but is the slowest choice.  The choice Query::QT_REAL
    // uses floating-point arithmetic, but is not robust in all cases.


    /// <summary>
    /// Construct convex hull of a set of 2D points
    /// HullIndices provides ordered indices of vertices of input points that form hull.
    /// </summary>
    public class ConvexHull2
    {
        //default tolerance is coupled to default clipper resolution
        //this is to ensure that "on line" vertices as output by clipper are treated as "on line" by the convex hulls as well
        public const double DefaultConvexHullTolerance = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE;

        IReadOnlyList<Vector2d> mVertices;
        int mNumVertices;
        int mNumSimplices;
        double mEpsilon;
        Vector2d.Information info;
        int[] mIndices;

        public int Dimension
        {
            get { return info.mDimension; }
        }

        /// <summary>
        /// Number of convex polygon edges
        /// </summary>
        public int NumSimplices
        {
            get { return mNumSimplices; }
        }

        /// <summary>
        ///   array of indices into V that represent the convex polygon edges (NumSimplices total elements)
        /// The i-th edge has vertices
        ///   vertex[0] = V[I[i]]
        ///   vertex[1] = V[I[(i+1) % SQ]]
        /// </summary>
        public int[] HullIndices
        {
            get { return mIndices; }
        }

        public Vector2d.Information DimensionalInformation { get => info; }


        /// <summary>
        /// Compute convex hull of input points. 
        /// epsilon is only used for check if points lie on a line (1d hull), not for rest of compute.
        /// </summary>
        public ConvexHull2(IReadOnlyList<Vector2d> vertices, double epsilon)
        {
            mVertices = vertices;
            mNumVertices = vertices.Count;
            mEpsilon = epsilon;

            Vector2d.GetInformation(mVertices, mEpsilon, out info);

            if (info.mDimension < 2)
                // The set is a point or (nearly) collinear.
                return;

            int i0 = info.mExtreme[0];
            int i1 = info.mExtreme[1];
            int i2 = info.mExtreme[2];

            Edge edge0;
            Edge edge1;
            Edge edge2;

            if (info.mExtremeCCW)
            {
                edge0 = new Edge(i0, i1);
                edge1 = new Edge(i1, i2);
                edge2 = new Edge(i2, i0);
            }
            else
            {
                edge0 = new Edge(i0, i2);
                edge1 = new Edge(i2, i1);
                edge2 = new Edge(i1, i0);
            }

            edge0.Insert(edge2, edge1);
            edge1.Insert(edge0, edge2);
            edge2.Insert(edge1, edge0);

            Edge hull = edge0;

            // ideally we insert points in random order. but instead of
            // generating a permutation, just insert them using modulo-indexing, 
            // which is in the ballpark...
            int ii = 0;
            do
            {
                if (!Update(ref hull, ii))
                    return;
                ii = (ii + 31337) % mNumVertices;
            } while (ii != 0);

            // original code, vastly slower in pathological cases
            //for (int i = 0; i < mNumVertices; ++i) {
            //    if ( ! Update(ref hull, i) )
            //        return;
            //}

            edge0 = hull;
            //remove colinear edges
            do
            {
                if (edge0.IsColinear(mVertices, mEpsilon))
                {
                    edge1 = edge0.E1;
                    if (edge1 == hull) hull = hull.E0;
                    edge1.RemoveSelf();
                }
                edge0 = edge0.E1;
            }
            while (edge0 != hull);

            GetIndices(hull);
        }

        private void GetIndices(Edge hull)
        {
            // Count the number of edge vertices and allocate the index array.
            mNumSimplices = 0;
            Edge current = hull;
            do
            {
                ++mNumSimplices;
                current = current.E1;
            } while (current != hull);

            mIndices = new int[mNumSimplices];

            // Fill the index array.
            mNumSimplices = 0;
            current = hull;
            do
            {
                mIndices[mNumSimplices] = current.V[0];
                ++mNumSimplices;
                current = current.E1;
            } while (current != hull);
        }

        /// <summary>
        /// Extract convex hull polygon from input points
        /// </summary>
        public Polygon2d GetHullPolygon()
        {
            Polygon2d poly2d = new Polygon2d();
            if (info.mDimension == 2)
            {
                var indices = HullIndices;
                Span<Vector2d> span = poly2d.VerticesAsSpanWithCount(indices.Length);
                for (int i = 0; i < span.Length; i++)
                {
                    span[i] = mVertices[indices[i]];
                }
                return poly2d;
            }
            else
            {
                poly2d.VerticesCapacity = 3;
                double cheatingDist = mEpsilon;
                if (info.mDimension == 1)
                {
                    //input is colinear
                    poly2d.AppendVertex(info.mMin);
                    poly2d.AppendVertex(info.mMax);
                    //yeah we are cheating
                    var notRealVert = info.mMax + poly2d.GetNormal(1) * cheatingDist;
                    poly2d.AppendVertex(notRealVert);
                }
                else
                {
                    //input is a point
                    var p = info.mMin;
                    poly2d.AppendVertex(p);
                    poly2d.AppendVertex(new Vector2d(p.x + cheatingDist, p.y));
                    poly2d.AppendVertex(new Vector2d(p.x, p.y + cheatingDist));
                }
                return poly2d;
            }
        }



        bool Update(ref Edge hull, int i)
        {
            // Locate an edge visible to the input point (if possible).
            Edge visible = null;
            Edge current = hull;
            do
            {
                //is index already part of the hull?
                if (current.V[0] == i || current.V[1] == i)
                    return true;
                if (current.IsConvex(i, mVertices, mEpsilon))
                {
                    visible = current;
                    break;
                }

                current = current.E1;
            }
            while (current != hull);

            if (visible == null)
            {
                // The point is inside the current hull; nothing to do.
                return true;
            }

            // Remove the visible edges.
            Edge adj0 = visible.E0;
            Debug.Assert(adj0 != null); // "Expecting nonnull adjacent\n");
            if (adj0 == null)
            {
                return false;
            }

            Edge adj1 = visible.E1;
            Debug.Assert(adj1 != null); // "Expecting nonnull adjacent\n");
            if (adj1 == null)
            {
                return false;
            }

            visible.DeleteSelf();

            while (adj0.IsConvex(i, mVertices, -mEpsilon))
            {
                hull = adj0;
                adj0 = adj0.E0;
                Debug.Assert(adj0 != null); // "Expecting nonnull adjacent\n");
                if (adj0 == null)
                {
                    return false;
                }

                adj0.E1.DeleteSelf();
            }

            while (adj1.IsConvex(i, mVertices, -mEpsilon))
            {
                hull = adj1;
                adj1 = adj1.E1;
                Debug.Assert(adj1 != null); // "Expecting nonnull adjacent\n");
                if (adj1 == null)
                {
                    return false;
                }

                adj1.E0.DeleteSelf();
            }

            // Insert the new edges formed by the input point and the end points of
            // the polyline of invisible edges.
            Edge edge0 = new Edge(adj0.V[1], i);
            Edge edge1 = new Edge(i, adj1.V[0]);
            edge0.Insert(adj0, edge1);
            edge1.Insert(edge0, adj1);
            hull = edge0;

            return true;
        }

        /// <summary>
        /// Internal class that represents edge of hull, and neighbors
        /// </summary>
        protected class Edge
        {
            public Vector2i V;
            public Edge E0;
            public Edge E1;
            public double sign;
            public int Time;

            public Edge(int v0, int v1)
            {
                sign = 0;
                Time = -1;
                V[0] = v0;
                V[1] = v1;
                E0 = null;
                E1 = null;
            }

            public bool IsConvex(int i, IReadOnlyList<Vector2d> vtcs, double tolerance)
            {
                if (i != Time)
                {
                    Time = i;
                    var center = vtcs[i];
                    var prev = vtcs[V.x];
                    var next = vtcs[V.y];
                    var cp = prev - center;
                    var cn = next - center;
                    sign = cp.Perp.Dot(cn);
                }
                return sign > tolerance;
            }

            public bool IsColinear(IReadOnlyList<Vector2d> vtcs, double tolerance)
            {
                var prev = vtcs[V.x];
                var center = vtcs[V.y];
                var next = vtcs[E1.V.y];
                var cp = prev - center;
                var cn = next - center;
                var angle = cp.Normalized.AngleR(cn.Normalized);
                return Math.Abs(angle - Math.PI) < tolerance;
            }

            public void Insert(Edge adj0, Edge adj1)
            {
                adj0.E1 = this;
                adj1.E0 = this;
                E0 = adj0;
                E1 = adj1;
            }

            public void RemoveSelf()
            {
                if (E0 != null)
                    E0.E1 = E1;
                if (E1 != null)
                    E1.E0 = E0;
            }

            public void DeleteSelf()
            {
                if (E0 != null)
                    E0.E1 = null;
                if (E1 != null)
                    E1.E0 = null;
            }
        }

    }
}
