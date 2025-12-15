using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace g3
{
    public class PolygonTree : IEnumerable<Polygon2d>
    {        
        public readonly List<GeneralPolygon2d> Childs = new();

        public class PolyTreeHole : Polygon2d, IEnumerable<Polygon2d>
        {
            public readonly List<GeneralPolygon2d> Childs = new();
            IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
            public IEnumerator<Polygon2d> GetEnumerator()
            {
                yield return this;
                using var enumerator = YieldRecursive(Childs);
                while (enumerator.MoveNext()) yield return enumerator.Current;
            }
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
        public IEnumerator<Polygon2d> GetEnumerator() => YieldRecursive(Childs);        
        private static IEnumerator<Polygon2d> YieldRecursive(IEnumerable<GeneralPolygon2d> genPolys)
        {
            foreach (var genPoly in genPolys)
            {
                yield return genPoly.Outer;
                foreach (Polygon2d hole in genPoly.Holes)
                {
                    if (hole is PolyTreeHole pTreeHole)
                    {
                        foreach (var child in pTreeHole)
                        {
                            yield return child;
                        }
                    }
                    else
                    {
                        yield return hole;
                    }
                }
            }
        }
    }

    public class GeneralPolygon2d : IDuplicatable<GeneralPolygon2d>, IEnumerable<Polygon2d>
    {
        protected Polygon2d outer;
        protected List<Polygon2d> holes = new List<Polygon2d>();
        bool bOuterIsCW;

        public GeneralPolygon2d()
        {
        }

        public GeneralPolygon2d(Polygon2d outer)
        {
            Outer = outer;
        }

        public GeneralPolygon2d(GeneralPolygon2d copy)
        {
            outer = copy.outer.Duplicate();
            bOuterIsCW = copy.bOuterIsCW;
            holes = new List<Polygon2d>();
            foreach (var hole in copy.holes)
                holes.Add(hole.Duplicate());
        }

        public virtual GeneralPolygon2d Duplicate()
        {
            return new GeneralPolygon2d(this);
        }

        public Polygon2d Outer
        {
            get { return outer; }
            set
            {
                outer = value;
                bOuterIsCW = outer.IsClockwise;
            }
        }

        public void AddHole(Polygon2d hole, bool bCheckContainment = true, bool bCheckOrientation = true)
        {
            if (outer == null)
                throw new Exception("GeneralPolygon2d.AddHole: outer polygon not set!");
            if (bCheckContainment)
            {
                if (outer.Contains(hole) == false)
                    throw new Exception("GeneralPolygon2d.AddHole: outer does not contain hole!");

                // [RMS] segment/segment intersection broken?
                foreach (var hole2 in holes)
                {
                    if (hole.Intersects(hole2))
                        throw new Exception("GeneralPolygon2D.AddHole: new hole intersects existing hole!");
                }
            }
            if (bCheckOrientation)
            {
                if ((bOuterIsCW && hole.IsClockwise) || (bOuterIsCW == false && hole.IsClockwise == false))
                    throw new Exception("GeneralPolygon2D.AddHole: new hole has same orientation as outer polygon!");
            }

            holes.Add(hole);
        }

        public void ClearHoles()
        {
            holes.Clear();
        }

        public bool HasHoles => holes.Count > 0;

        public virtual IReadOnlyList<Polygon2d> Holes
        {
            get { return holes; }
        }

        public double Area
        {
            get
            {
                double sign = (bOuterIsCW) ? -1.0 : 1.0;
                double dArea = sign * Outer.SignedArea;
                foreach (var h in holes)
                    dArea += sign * h.SignedArea;
                return dArea;
            }
        }

        public double HoleArea
        {
            get
            {
                double dArea = 0;
                foreach (var h in Holes)
                    dArea += Math.Abs(h.SignedArea);
                return dArea;
            }
        }

        public double Perimeter
        {
            get
            {
                double dPerim = outer.Perimeter;
                foreach (var h in holes)
                    dPerim += h.Perimeter;
                return dPerim;
            }
        }

        public AxisAlignedBox2d Bounds
        {
            get
            {
                AxisAlignedBox2d box = outer.Bounds;
                foreach (var h in holes)
                    box.Contain(h.Bounds);
                return box;
            }
        }

        public int VertexCount
        {
            get
            {
                int NV = outer.VertexCount;
                foreach (var h in holes)
                    NV += h.VertexCount;
                return NV;
            }
        }

        public void Translate(Vector2d translate)
        {
            outer.Translate(translate);
            foreach (var h in holes)
                h.Translate(translate);
        }

        public void Rotate(Matrix2d rotation, Vector2d origin)
        {
            outer.Rotate(rotation, origin);
            foreach (var h in holes)
                h.Rotate(rotation, origin);
        }

        public void Scale(Vector2d scale, Vector2d origin)
        {
            outer.Scale(scale, origin);
            foreach (var h in holes)
                h.Scale(scale, origin);
        }

        public void Transform(Func<Vector2d, Vector2d> transformF)
        {
            outer.Transform(transformF);
            foreach (var h in holes)
                h.Transform(transformF);
        }

        public void Reverse()
        {
            Outer.Reverse();
            bOuterIsCW = Outer.IsClockwise;
            foreach (var h in Holes)
                h.Reverse();
        }

        public bool Contains(Vector2d vTest)
        {
            if (outer.Contains(vTest) == false)
                return false;
            foreach (var h in holes)
            {
                if (h.Contains(vTest))
                    return false;
            }
            return true;
        }

        public bool Contains(Circle2d circle)
        {
            if (Outer.Contains(circle))
            {
                foreach (var hole in Holes)
                {
                    if (!circle.IsOutside(hole))
                        return false;
                }
                return true;
            }
            else return false;
        }

        /// <summary>
        /// Returns true if point P is inside the polygon or on the boundary (inclusive contains with epsilon tolerance)
        /// of the polygon, using fast winding-number computation.
        /// </summary>
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// <param name="epsilon">tolerance for point on edge check</param>
        /// <returns></returns>
        public bool ContainsInclusive(Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            if (outer.ContainsInclusive(P, epsilon) == false)
                return false;
            foreach (var h in holes)
            {
                if (h.ContainsInclusive(P, epsilon))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Returns true if point P is inside the polygon, but not on the boundary (exclusive contains with epsilon tolerance)
        /// of the polygon, using fast winding-number computation.
        /// </summary>
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// <param name="epsilon">tolerance for point on edge check</param>
        /// <returns></returns>
        public bool ContainsExclusive(Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            if (outer.ContainsExclusive(P, epsilon) == false)
                return false;
            foreach (var h in holes)
            {
                if (h.ContainsExclusive(P, epsilon))
                    return false;
            }
            return true;
        }

        public bool Contains(Polygon2d poly)
        {
            if (outer.Contains(poly) == false)
                return false;
            foreach (var h in holes)
            {
                if (h.Contains(poly))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Checks that all points on a segment are within the area defined by the GeneralPolygon2d;
        /// holes are included in the calculation.
        /// </summary>
        public bool Contains(Segment2d seg)
        {
            if (outer.Contains(seg) == false)
                return false;
            foreach (var h in holes)
            {
                if (h.Intersects(seg))
                    return false;
            }
            return true;
        }

        public bool Intersects(Polygon2d poly)
        {
            if (outer.Intersects(poly))
                return true;
            foreach (var h in holes)
            {
                if (h.Intersects(poly))
                    return true;
            }
            return false;
        }

        public Vector2d PointAt(int iSegment, double fSegT, int iHoleIndex = -1)
        {
            if (iHoleIndex == -1)
                return outer.PointAt(iSegment, fSegT);
            return holes[iHoleIndex].PointAt(iSegment, fSegT);
        }

        public Segment2d Segment(int iSegment, int iHoleIndex = -1)
        {
            if (iHoleIndex == -1)
                return outer.Segment(iSegment);
            return holes[iHoleIndex].Segment(iSegment);
        }

        public Vector2d GetNormal(int iSegment, double segT, int iHoleIndex = -1)
        {
            if (iHoleIndex == -1)
                return outer.GetNormal(iSegment, segT);
            return holes[iHoleIndex].GetNormal(iSegment, segT);
        }

        // this should be more efficient when there are holes...
        public double DistanceSquared(Vector2d p, out int iHoleIndex, out int iNearSeg, out double fNearSegT)
        {
            iNearSeg = iHoleIndex = -1;
            fNearSegT = double.MaxValue;
            double dist = outer.DistanceSquared(p, out iNearSeg, out fNearSegT);
            for (int i = 0; i < Holes.Count; ++i)
            {
                int seg; double segt;
                double holedist = Holes[i].DistanceSquared(p, out seg, out segt);
                if (holedist < dist)
                {
                    dist = holedist;
                    iHoleIndex = i;
                    iNearSeg = seg;
                    fNearSegT = segt;
                }
            }
            return dist;
        }

        public IEnumerable<Segment2d> AllSegmentsItr()
        {
            foreach (Segment2d seg in outer.SegmentItr())
                yield return seg;
            foreach (var hole in holes)
            {
                foreach (Segment2d seg in hole.SegmentItr())
                    yield return seg;
            }
        }

        public IEnumerable<Vector2d> AllVerticesItr()
        {
            foreach (Vector2d v in outer.Vertices)
                yield return v;
            foreach (var hole in holes)
            {
                foreach (Vector2d v in hole.Vertices)
                    yield return v;
            }
        }

        public void Simplify(double clusterTol = 0.0001,
                              double lineDeviationTol = 0.01,
                              bool bSimplifyStraightLines = true)
        {
            // [TODO] should make sure that holes stay inside Outer!!
            Outer.Simplify(clusterTol, lineDeviationTol, bSimplifyStraightLines);
            foreach (var hole in holes)
                hole.Simplify(clusterTol, lineDeviationTol, bSimplifyStraightLines);
        }

        public IEnumerator<Polygon2d> GetEnumerator()
        {
            yield return outer;
            foreach (var hole in holes) yield return hole;
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
}
