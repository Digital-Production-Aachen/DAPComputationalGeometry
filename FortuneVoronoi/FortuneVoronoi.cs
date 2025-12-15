using g3;
using System;
using System.Collections.Generic;

namespace FortuneVoronoi
{
    public class VoronoiGraph
    {
        internal readonly HashSet<Vector2d> _mutableVertices = new HashSet<Vector2d>();
        public IReadOnlyCollection<Vector2d> Vertices => _mutableVertices;

        internal readonly HashSet<Edge> _mutableEdges = new HashSet<Edge>();
        private bool edgesCleanedUp = false;
        public IReadOnlyCollection<Edge> Edges
        {
            get
            {
                //lazily clean up the edges only when they are accessed, to avoid this work if only vertices are needed
                if (!edgesCleanedUp)
                {
                    foreach (var ve in _mutableEdges)
                    {
                        if (ve.Done)
                            continue;
                        if (!ve.VVertexB.HasValue)
                        {
                            ve.AddVertex(FortuneVoronoiAlgorithm.VvInfinite);
                            if (Math.Abs(ve.LeftData.y - ve.RightData.y) < 1e-10 && ve.LeftData.x < ve.RightData.x)
                            {
                                var t = ve.LeftData;
                                ve.LeftData = ve.RightData;
                                ve.RightData = t;
                            }
                        }
                    }

                    var minuteEdges = new List<Edge>();
                    foreach (var ve in _mutableEdges)
                    {
                        if (!ve.IsPartlyInfinite && ve.VVertexA.Equals(ve.VVertexB))
                        {
                            minuteEdges.Add(ve);
                            // prevent rounding errors from expanding to holes
                            foreach (var ve2 in _mutableEdges)
                            {
                                if (ve2.VVertexA.Equals(ve.VVertexA))
                                    ve2.VVertexA = ve.VVertexA;
                                if (ve2.VVertexB.Equals(ve.VVertexA))
                                    ve2.VVertexB = ve.VVertexA;
                            }
                        }
                    }
                    _mutableEdges.ExceptWith(minuteEdges);

                    edgesCleanedUp = true;
                }

                return _mutableEdges;
            }
        }

        [System.Diagnostics.CodeAnalysis.Experimental("IS_STILL_BUGGY")]
        public VoronoiGraph(IEnumerable<Vector2d> points)
        {
            // build the Voronoi graph using Fortunes algorithm
            var pq = new PriorityQueue<VEvent, VEvent>();
            var currentCircles = new Dictionary<VDataNode, VCircleEvent>();
            VNode rootNode = null;
            foreach (var v in points)
            {
                var dataEvent = new VDataEvent(v);
                pq.Enqueue(dataEvent, dataEvent);
            }
            while (pq.Count > 0)
            {
                var ve = pq.Dequeue();
                VDataNode[] circleCheckList;
                if (ve is VDataEvent dataEvent)
                {
                    rootNode = VNode.ProcessDataEvent(dataEvent, rootNode, this, ve.Y, out circleCheckList);
                }
                else if (ve is VCircleEvent circleEvent)
                {
                    currentCircles.Remove(circleEvent.NodeN);
                    if (!circleEvent.Valid)
                        continue;
                    rootNode = VNode.ProcessCircleEvent(circleEvent, rootNode, this, out circleCheckList);
                }
                else
                    throw new Exception("Got event of type " + ve.GetType() + "!");

                foreach (var vd in circleCheckList)
                {
                    if (currentCircles.ContainsKey(vd))
                    {
                        currentCircles[vd].Valid = false;
                        currentCircles.Remove(vd);
                    }
                    var vce = VNode.CircleCheckDataNode(vd, ve.Y);
                    if (vce != null)
                    {
                        pq.Enqueue(vce, vce);
                        currentCircles[vd] = vce;
                    }
                }

                var evt = ve as VDataEvent;
                if (evt != null)
                {
                    var dp = evt.DataPoint;
                    foreach (var vce in currentCircles.Values)
                    {
                        double distToCenter = dp.Distance(vce.Center);
                        if (distToCenter < vce.Y - vce.Center.y && Math.Abs(distToCenter - (vce.Y - vce.Center.y)) > 1e-10)
                            vce.Valid = false;
                    }
                }
            }

            VNode.CleanUpTree(rootNode as VEdgeNode);
        }

        public IEnumerable<Segment2d> EdgesAsSegments(AxisAlignedBox2d drawBounds)
        {
            foreach(var edge in Edges)
            {
                double length = edge.Length;
                Vector2d center = edge.FixedPoint;
                Vector2d dir = edge.DirectionVector;
                if (length == double.PositiveInfinity)
                {
                    //is a ray
                    var line = new Line2d(center, dir);
                    Vector2d targetCorner = dir.x < 0 ? drawBounds.Min : drawBounds.Max;
                    
                }else
                yield return new Segment2d(center, center + dir * length);
            }
        }

        internal static class FortuneVoronoiAlgorithm
        {
            internal static readonly Vector2d VvInfinite = new Vector2d(float.PositiveInfinity, float.PositiveInfinity);

            internal static double ParabolicCut(double x1, double y1, double x2, double y2, double ys)
            {
                if (Math.Abs(x1 - x2) < 1e-10 && Math.Abs(y1 - y2) < 1e-10)
                    throw new ArgumentException("coincident points");

                if (Math.Abs(y1 - ys) < 1e-10 && Math.Abs(y2 - ys) < 1e-10)
                    return (x1 + x2) / 2;
                if (Math.Abs(y1 - ys) < 1e-10)
                    return x1;
                if (Math.Abs(y2 - ys) < 1e-10)
                    return x2;
                var a1 = 1 / (2 * (y1 - ys));
                var a2 = 1 / (2 * (y2 - ys));
                if (Math.Abs(a1 - a2) < 1e-10)
                    return (x1 + x2) / 2;

                double a1a2 = a1 * a2;
                double a1x1 = a1 * x1;
                double a2x2 = a2 * x2;
                double a1x1a2x2_4dif = 4 * a1x1 - 4 * a2x2;
                var xs1 = 0.5 / (2 * a1 - 2 * a2) * (a1x1a2x2_4dif + 2 * Math.Sqrt(-8 * a1x1 * a2x2 - 2 * a1 * y1 + 2 * a1 * y2 + 4 * a1a2 * x2 * x2 + 2 * a2 * y1 + 4 * a1a2 * x1 * x1 - 2 * a2 * y2));
                var xs2 = 0.5 / (2 * a1 - 2 * a2) * (a1x1a2x2_4dif - 2 * Math.Sqrt(-8 * a1x1 * a2x2 - 2 * a1 * y1 + 2 * a1 * y2 + 4 * a1a2 * x2 * x2 + 2 * a2 * y1 + 4 * a1a2 * x1 * x1 - 2 * a2 * y2));
                xs1 = Math.Round(xs1, 10);
                xs2 = Math.Round(xs2, 10);
                if (xs1 > xs2)
                {
                    var h = xs1;
                    xs1 = xs2;
                    xs2 = h;
                }
                if (y1 >= y2)
                    return xs2;
                return xs1;
            }

            internal static Vector2d CircumCircleCenter(Vector2d a, Vector2d b, Vector2d c)
            {
                if (a == b || b == c || a == c)
                    throw new Exception("Need three different points!");

                Vector2d t = (a + c) / 2;
                Vector2d v = (b + c) / 2;

                double ux, uy, wx, wy;

                if (a.x == c.x)
                {
                    ux = 1;
                    uy = 0;
                }
                else
                {
                    ux = (c.y - a.y) / (a.x - c.x);
                    uy = 1;
                }

                if (b.x == c.x)
                {
                    wx = -1;
                    wy = 0;
                }
                else
                {
                    wx = (b.y - c.y) / (b.x - c.x);
                    wy = -1;
                }

                var alpha = (wy * (v.x - t.x) - wx * (v.y - t.y)) / (ux * wy - wx * uy);

                return new Vector2d(t.x + alpha * ux, t.y + alpha * uy);
            }
        }
    }
}