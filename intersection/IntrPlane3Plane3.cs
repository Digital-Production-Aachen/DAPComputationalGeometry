using System;
using System.Diagnostics;
using System.Diagnostics.SymbolStore;

namespace g3
{
    // ported from Bery0za/geometry3Sharp
    // ported from WildMagic5 IntrPlane3Plane3
    // use Test() for fast boolean query, does not compute intersection info
    // use Find() to compute full information
    public class IntrPlane3Plane3
    {
        Plane3d plane0;
        public Plane3d Plane0
        {
            get { return plane0; }
            set { plane0 = value; Result = IntersectionResult.NotComputed; }
        }

        Plane3d plane1;
        public Plane3d Plane1
        {
            get { return plane1; }
            set { plane1 = value; Result = IntersectionResult.NotComputed; }
        }

        // result flags
		public int Quantity = 0;
		public IntersectionResult Result = IntersectionResult.NotComputed;
		public IntersectionType Type = IntersectionType.Empty;

        // only valid if intersection type is Line
        public Line3d Line;

        // only valid if intersection type is Plane
        public Plane3d Plane;


		public IntrPlane3Plane3(Plane3d p0, Plane3d p1)
		{
			plane0 = p0;
            plane1 = p1;
		}


        public IntrPlane3Plane3 Compute()
        {
            Find();
            return this;
        }

        // If N0 and N1 are parallel, either the planes are parallel and separated
        // or the same plane.  In both cases, 'false' is returned.  Otherwise,
        // the intersection line is
        //   L(t) = t*Cross(N0,N1)/|Cross(N0,N1)| + c0*N0 + c1*N1
        // for some coefficients c0 and c1 and for t any real number (the line
        // parameter).  Taking dot products with the normals,
        //   d0 = Dot(N0,L) = c0*Dot(N0,N0) + c1*Dot(N0,N1) = c0 + c1*d
        //   d1 = Dot(N1,L) = c0*Dot(N0,N1) + c1*Dot(N1,N1) = c0*d + c1
        // where d = Dot(N0,N1).  These are two equations in two unknowns.  The
        // solution is
        //   c0 = (d0 - d*d1)/det
        //   c1 = (d1 - d*d0)/det
        // where det = 1 - d^2.
        public bool Find()
        {
            if (Result != IntersectionResult.NotComputed)
                return Result != IntersectionResult.NoIntersection;

            double dot = plane0.Normal.Dot(plane1.Normal);
            if (Math.Abs(dot) >= 1 - MathUtil.ZeroTolerance)
            {
                double cDiff = dot >= 0 ? Plane0.Constant - Plane1.Constant : Plane0.Constant + Plane1.Constant;

                if (Math.Abs(cDiff) < MathUtil.ZeroTolerance)
                {
                    Type = IntersectionType.Plane;
                    Result = IntersectionResult.NoIntersection;
                    Plane = new Plane3d(plane0.Normal, plane0.Constant);
                }

                Type = IntersectionType.Empty;
                Result = IntersectionResult.NoIntersection;
                return false;
            }
            
            double invDet = 1 / (1 - dot * dot);
            double c0 = (plane0.Constant - dot * plane1.Constant) * invDet;
            double c1 = (plane1.Constant - dot * plane0.Constant) * invDet;
            Quantity = 1;
            Type = IntersectionType.Line;
            Line = new Line3d(c0 * plane0.Normal + c1 * plane1.Normal, plane0.Normal.UnitCross(plane1.Normal));
            return true;
        }

        // If Cross(N0,N1) is zero, then either planes are parallel and separated
        // or the same plane.  In both cases, 'false' is returned.  Otherwise, the
        // planes intersect.  To avoid subtle differences in reporting between
        // Test() and Find(), the same parallel test is used.  Mathematically,
        //   |Cross(N0,N1)|^2 = Dot(N0,N0)*Dot(N1,N1)-Dot(N0,N1)^2
        //                    = 1 - Dot(N0,N1)^2
        // The last equality is true since planes are required to have unit-length
        // normal vectors.  The test |Cross(N0,N1)| = 0 is the same as
        // |Dot(N0,N1)| = 1.  I test the latter condition in Test() and Find().
        public bool Test()
        {
            double dot = plane0.Normal.Dot(plane1.Normal);
            return Math.Abs(dot) < 1 - MathUtil.ZeroTolerance;
        }
    }
}
