using FortuneVoronoi;
using System;

namespace g3
{
    public static class LargestInscribedCircleFortune
    {
        [System.Diagnostics.CodeAnalysis.Experimental("IS_STILL_BUGGY")]
        public static Circle2d LargestInscribedCircle(this GeneralPolygon2d polygon)
        {
            Vector2d center = Vector2d.Zero;
            double radiusSquared = 0;

            var voronoiGraph = new VoronoiGraph(polygon.AllVerticesItr());

            foreach (var vertex in voronoiGraph.Vertices)
            {
                bool isCandidate = polygon.DistanceSquaredGreaterThan(vertex, radiusSquared, out double distSqrd, out int iHole, out int iNearSeg, out double fNearSeg);
                if (isCandidate && polygon.Contains(vertex))
                {
                    center = vertex;
                    radiusSquared = distSqrd;
                }
            }

            return new Circle2d(center, Math.Sqrt(radiusSquared));
        }
    }
}
