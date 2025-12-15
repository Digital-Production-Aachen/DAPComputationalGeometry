using g3;
using System;

namespace FortuneVoronoi
{
    public class Edge
    {
        internal bool Done;

        public Vector2d RightData { get; internal set; }
        public Vector2d LeftData { get; internal set; }

        internal Vector2d? VVertexA { get; set; }
        internal Vector2d? VVertexB { get; set; }

        internal void AddVertex(Vector2d v)
        {
            if (!VVertexA.HasValue)
                VVertexA = v;
            else if (!VVertexB.HasValue)
                VVertexB = v;
            else
                throw new Exception("Tried to add third vertex!");
        }

        internal bool IsInfinite
        {
            get { return !VVertexA.Value.IsFinite && !VVertexB.Value.IsFinite; }
        }

        internal bool IsPartlyInfinite
        {
            get { return !VVertexA.Value.IsFinite || !VVertexB.Value.IsFinite; }
        }

        public Vector2d FixedPoint
        {
            get
            {
                if (!VVertexA.HasValue || !VVertexB.HasValue)
                    throw new InvalidOperationException("Cannot get FixedPoint from edge which is not completely initialised");

                if (IsInfinite)
                    return 0.5f * (LeftData + RightData);

                if (VVertexA.Value.IsFinite)
                    return VVertexA.Value;

                return VVertexB.Value;
            }
        }

        public Vector2d DirectionVector
        {
            get
            {
                if (!VVertexA.HasValue || !VVertexB.HasValue)
                    throw new InvalidOperationException("Cannot get DirectionVector from edge which is not completely initialised");

                if (!IsPartlyInfinite)
                    return (VVertexB.Value - VVertexA.Value).Normalized;

                if (Math.Abs(LeftData.x - RightData.x) < float.Epsilon)
                {
                    if (LeftData.y < RightData.y)
                        return new Vector2d(-1, 0);
                    return new Vector2d(1, 0);
                }

                var erg = new Vector2d(-(RightData.y - LeftData.y) / (RightData.x - LeftData.x), 1);
                if (RightData.x < LeftData.x)
                    erg *= -1;
                erg *= 1.0f / erg.Length;
                return erg;
            }
        }

        public double Length
        {
            get
            {
                if (!VVertexA.HasValue || !VVertexB.HasValue)
                    throw new InvalidOperationException("Cannot get Length from edge which is not completely initialised");

                if (IsPartlyInfinite)
                    return double.PositiveInfinity;
                return VVertexA.Value.Distance(VVertexB.Value);
            }
        }
    }
}
