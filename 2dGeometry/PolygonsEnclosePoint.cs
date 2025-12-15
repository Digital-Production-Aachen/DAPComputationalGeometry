using g3;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace g3
{
    public class PolygonsEnclosePoint
    {
        public Vector2d Point { get => _point; }
        public double Tolerance { get; }
        public bool UseThreadLocalClipper { get; }
        public AxisAlignedBox2d WorldBounds { get; }

        public int VertexCountThreshold { get; }

        public IReadOnlyCollection<int> MinimumBlockerSet => minBlockers;

        readonly HashSet<int> minBlockers = new();
        public Polygon2d ContainingHole { get; private set; }

        public PolygonsEnclosePoint(Vector2d point, double tolerance = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE, bool useThreadLocalClipper = false, AxisAlignedBox2d worldBounds = default, int vertexCountThreshold = 100)
        {
            //round point to closest clipper int point for exact comparisons
            _point = point.ToIntPoint().ToVector2d();
            Tolerance = tolerance;
            UseThreadLocalClipper = useThreadLocalClipper;
            WorldBounds = worldBounds.Area > 0 ? worldBounds : Clipper2Wrapper.CoordinateMax;
            VertexCountThreshold = vertexCountThreshold;
        }

        private struct Quadrant
        {
            public Polygon2d[] curUnion;
            public readonly Queue<(int key, Polygon2d poly)> backlog;
            public readonly List<int> minBlockers;
            public int holeIdx;
            public bool IsBlocked => holeIdx != -1;
            public Polygon2d Hole => IsBlocked ? curUnion[holeIdx] : null;

            public Quadrant()
            {
                backlog = new();
                minBlockers = new();
                holeIdx = -1;
            }

            public void Enqueue(int key, Polygon2d poly)
            {
                backlog.Enqueue((key, poly));
            }

            public int VertexCountForUnion
            {
                get
                {
                    int sum = 0;
                    if (curUnion != null)
                        for (int i = 0; i < curUnion.Length; i++) sum += curUnion[i].VertexCount;
                    foreach (var kvp in backlog) sum += kvp.poly.VertexCount;
                    return sum;
                }
            }
        }

        private ref Quadrant GetQuadrant(int key)
        {
            switch (key)
            {
                case 0: return ref NE;
                case 1: return ref NW;
                case 2: return ref SW;
                case 3: return ref SE;
                default: throw new IndexOutOfRangeException();
            }
        }

        //4 quadrants for cardinal directions NESW
        Quadrant NE = new();
        Quadrant NW = new();
        Quadrant SW = new();
        Quadrant SE = new();
        const int numQ = 4;
        State state = State.EMPTY;
        private Vector2d _point;

        public enum EnclosureResult
        {
            NOT_ENCLOSED,
            ENCLOSED,
            INSIDE_POLY,
        }

        private enum State
        {
            /// <summary>
            /// No polygons added yet
            /// </summary>
            EMPTY,
            /// <summary>
            /// we use only north-east [0] as "central" quadrant 
            /// as long as we don't have any polys that we could skip based on bounds 
            /// </summary>
            NE_IS_CENTER,
            /// <summary>
            /// at least one quadrant is not blocked, can skip polys based on bounds
            /// </summary>
            QUADRANTS,
            /// <summary>
            /// all quadrants are blocked, but there is an escape path crossing multiple quadrants
            /// we use north-east [0] as "global" union 
            /// </summary>
            ALL_QUADRANTS_BLOCKED,
            /// <summary>
            /// proven blocked
            /// </summary>
            BLOCKED,
        }

        public void Clear(Vector2d newPoint)
        {
            for(int i = 0; i < numQ; i++)
            {
                ref Quadrant quad = ref GetQuadrant(i);
                quad.curUnion = null;
                quad.backlog?.Clear();
                quad.minBlockers?.Clear();
                quad.holeIdx = -1;
            }
            minBlockers.Clear();
            ContainingHole = null;
            state = State.EMPTY;
            _point = newPoint;
        }

        public EnclosureResult AddEnclosurePolygons(int key, Polygon2d[] polygonWithHoles)
        {
            int containmentIdx = polygonWithHoles.IndexOfContainingPoly(Point, Tolerance);
            if (containmentIdx != -1)
            {
                //point is contained inside polygonWithHoles
                ContainingHole = polygonWithHoles[containmentIdx];
                minBlockers.Clear();
                minBlockers.Add(key);
                return ContainingHole.IsHole ? EnclosureResult.ENCLOSED : EnclosureResult.INSIDE_POLY;
            }

            if (state == State.BLOCKED)
                return EnclosureResult.ENCLOSED;

            ref Quadrant Q0 = ref GetQuadrant(0);//NE
            if (state == State.EMPTY)
            {
                Q0.curUnion = polygonWithHoles;
                Q0.minBlockers.Add(key);
                state = State.NE_IS_CENTER;
                return EnclosureResult.NOT_ENCLOSED;
            }

            foreach (Polygon2d polygon in polygonWithHoles)
            {
                //after we checked hole containment, we can ignore all holes for escape path analysis
                //because they cannot be reached from the outside
                if (polygon.IsHole) continue;
                AddToQuadrants(key, polygon);
            }

            switch (state)
            {
                //states using only quadrant 0 and global union
                case State.NE_IS_CENTER:
                case State.ALL_QUADRANTS_BLOCKED:
                    bool allBlocked = QuadrantBlocked(ref Q0);
                    if (allBlocked)
                    {
                        state = State.BLOCKED;
                        minBlockers.UnionWith(Q0.minBlockers);
                        ContainingHole = Q0.Hole;
                    }
                    return allBlocked ? EnclosureResult.ENCLOSED : EnclosureResult.NOT_ENCLOSED;
                //bounds checked quadrants in use
                case State.QUADRANTS:
                    Span<int> quadrants = stackalloc int[numQ];
                    Span<int> quadrantVertexCounts = stackalloc int[numQ];
                    for (int i = 0; i < quadrants.Length; i++)
                    {
                        quadrants[i] = i;
                        quadrantVertexCounts[i] = GetQuadrant(i).VertexCountForUnion;
                    }
                    quadrantVertexCounts.Sort(quadrants);
                    for (int i = 0; i < quadrants.Length; i++)
                    {
                        ref Quadrant qd = ref GetQuadrant(quadrants[i]);
                        bool blocked = QuadrantBlocked(ref qd);
                        if (!blocked) return EnclosureResult.NOT_ENCLOSED;
                    }

                    TransitionQ0ToAllBlocked();
                    state = State.ALL_QUADRANTS_BLOCKED;
                    goto case State.ALL_QUADRANTS_BLOCKED;

                default: throw new InvalidOperationException($"state is invalid: {state}");
            }
        }
        private void AddToQuadrants(int key, Polygon2d polygon)
        {
            AxisAlignedBox2d bounds = polygon.Bounds;
            switch (state)
            {
                case State.NE_IS_CENTER:
                    int q0verts = GetQuadrant(0).curUnion?.Sum(x => x.VertexCount) ?? 0;
                    if (q0verts < VertexCountThreshold || bounds.ContainsInclusive(Point, -Tolerance))
                    {
                        GetQuadrant(0).Enqueue(key, polygon);
                        return;
                    }
                    else
                    {
                        //init quadrants with union of centered and the quadrant cover poly
                        InitQuadrantsFromQ0();
                        state = State.QUADRANTS;
                        goto case State.QUADRANTS;
                    }
                case State.QUADRANTS:
                    ForQuadrantsInBounds(bounds, (ref Quadrant qd) => qd.Enqueue(key, polygon));
                    break;
                case State.ALL_QUADRANTS_BLOCKED:
                    GetQuadrant(0).Enqueue(key, polygon);
                    break;
            }
        }

        private void ForQuadrantsInBounds(in AxisAlignedBox2d bounds, QuadrantAction action)
        {
            if (bounds.Max.x >= Point.x)
            {
                if (bounds.Max.y >= Point.y) action(ref NE);
                if (bounds.Min.y <= Point.y) action(ref SE);
            }
            if (bounds.Min.x <= Point.x)
            {
                if (bounds.Max.y >= Point.y) action(ref NW);
                if (bounds.Min.y <= Point.y) action(ref SW);
            }
        }

        delegate void QuadrantAction(ref Quadrant qd);

        private bool QuadrantBlocked(ref Quadrant qd)
        {
            if (qd.IsBlocked) return true;
            Clipper2 clipper = UseThreadLocalClipper ? Clipper2.ThreadLocalInstance : new Clipper2();
            while (qd.backlog.Count > 0)
            {
                foreach (Polygon2d poly in qd.curUnion)
                {
                    //skip holes that Point is not contained in
                    //they are irrelevant for escape
                    if (poly.IsHole && !Touches2Quadrants(poly)) continue;
                    clipper.AddPolygonSubject(poly);
                }
                (int key, Polygon2d newPoly) = qd.backlog.Dequeue();
                clipper.AddPolygonSubject(newPoly);
                clipper.Union();
                qd.curUnion = clipper.Solution;
                clipper.Clear();

                if (qd.minBlockers[^1] != key)
                    qd.minBlockers.Add(key);
                qd.holeIdx = qd.curUnion.IndexOfContainingPoly(Point, Tolerance);
                if (qd.holeIdx != -1) return true;
            }
            
            return false;
        }

        private bool Touches2Quadrants(Polygon2d polygon)
        {
            bool seenX = false, seenY = false;
            var verts = polygon.VerticesAsReadOnlySpan;
            for (int i = 0; i < verts.Length; i++)
            {
                seenX |= AEql(verts[i].x, Point.x);
                seenY |= AEql(verts[i].y, Point.y);
            }
            return seenX && seenY;
        }

        private bool ContainsQuadrantEdge(Polygon2d polygon)
        {
            var verts = polygon.VerticesAsReadOnlySpan;
            Vector2d prev = verts[^1];
            for (int j = 0; j < verts.Length; j++)
            {
                Vector2d cur = verts[j];
                if (AEql(cur.x, Point.x) && AEql(prev.x, cur.x))
                {
                    return true;
                }
                if (AEql(cur.y, Point.y) && AEql(prev.y, cur.y))
                {
                    return true;
                }
                prev = cur;
            }
            return false;
        }

        //approximately equal
        private static bool AEql(double n1, double n2) => Math.Abs(n1 - n2) < g3.Constants.DEFAULT_LINE_IDENTITY_TOLERANCE;

        private static readonly Polygon2d EmptyPoly = new Polygon2d();
        #region transitions
        private void InitQuadrantsFromQ0()
        {
            ref Quadrant Q0 = ref GetQuadrant(0);//NE
            //union backlog before Init
            if (Q0.backlog.Count > 0) QuadrantBlocked(ref Q0);
            //add the cover with the last existing blocker, so that we do not add another (gets deduplicated)
            Q0.backlog.Enqueue((Q0.minBlockers[^1], QuadrantCoverPoly(0)));
            Polygon2d[] union = Q0.curUnion;
            for (int i = 0; i < numQ; i++)
            {
                ref Quadrant Qi = ref GetQuadrant(i);
                Qi.curUnion = new Polygon2d[union.Length];
                Array.Fill(Qi.curUnion, EmptyPoly);
                if (i == 0) continue;
                Qi.minBlockers.AddRange(Q0.minBlockers);
                Qi.holeIdx = Q0.holeIdx;
                Qi.backlog.Enqueue((Qi.minBlockers[^1], QuadrantCoverPoly(i)));
            }
            for (int i = 0; i < union.Length; i++)
            {
                Polygon2d poly = union[i];
                ForQuadrantsInBounds(poly.Bounds, (ref Quadrant qd) => qd.curUnion[i] = poly);
            }
        }

        private void TransitionQ0ToAllBlocked()
        {
            //use a tiny square hole around the Point to merge "point contact" holes into a single poly
            Polygon2d squareHoleAroundPoint = SetTrHoleVtcs();
            ref Quadrant Q0 = ref GetQuadrant(0);
            var clipper = UseThreadLocalClipper ? Clipper2.ThreadLocalInstance : new Clipper2();
            bool hasHole = false;
            for (int i = 0; i < numQ; i++)
            {
                ref Quadrant qd = ref GetQuadrant(i);
                minBlockers.UnionWith(qd.minBlockers);
                if (!qd.Hole.IsHole) continue;
                hasHole = true;
                clipper.AddPolygonSubject(qd.Hole);
            }

            if (hasHole)
            {
                clipper.AddPolygonSubject(squareHoleAroundPoint);
                clipper.Union();
                Polygon2d[] hole = clipper.Solution;
                foreach (var p in hole) p.RemoveConsecutiveCoincidentVertices();
                clipper.Clear();

                //shortcut: if one or more quadrants had holes and the result hole does not contain edges 
                //colinear to the quadrant boundaries, the hole we stitched together is enclosing Point
                if (hole.Length == 1 && !ContainsQuadrantEdge(hole[0]))
                {
                    hole[0].Reverse();
                    Q0.curUnion = hole;
                    Q0.holeIdx = 0;
                    return;
                }
            }

            for (int i = 0; i < numQ; i++)
            {
                ref Quadrant qd = ref GetQuadrant(i);
                clipper.AddPolygonSubject(qd.curUnion);
                clipper.AddPolygonClip(QuadrantCoverPoly(i));
                clipper.Difference();
                if (i == 0) continue;

                while (qd.backlog.Count > 0)
                    Q0.backlog.Enqueue(qd.backlog.Dequeue());
            }
            clipper.FlushBufferAddAsSubjects();
            clipper.AddPolygonClip(squareHoleAroundPoint);
            clipper.Difference();
            Q0.curUnion = clipper.Solution;
            Q0.holeIdx = Q0.curUnion.IndexOfContainingHole(Point);
            clipper.Clear();
        }

        private Polygon2d _triHole = new Polygon2d(3);
        private Polygon2d SetTrHoleVtcs()
        {
            const double sqrL = g3.Constants.DEFAULT_LINE_IDENTITY_TOLERANCE * 10;
            var vertices = _triHole.VerticesAsSpanWithCount(3);
            vertices[0] = new Vector2d(Point.x + sqrL / 2.0, Point.y);
            //vertices[1] = new Vector2d(Point.x + sqrL / 2.0, Point.y + sqrL / 2.0);
            vertices[1] = new Vector2d(Point.x - sqrL / 2.0, Point.y - sqrL / 2.0);
            vertices[2] = new Vector2d(Point.x - sqrL / 2.0, Point.y + sqrL / 2.0);
            return _triHole;
        }

        private Polygon2d QuadrantCoverPoly(int quadNr)
        {
            Polygon2d quadrantCoverPoly = new();
            Span<Vector2d> span = quadrantCoverPoly.VerticesAsSpanWithCount(6);
            switch (quadNr)
            {
                case 0://NE
                    span[0] = WorldBounds.GetCorner(0);
                    span[1] = WorldBounds.GetCorner(1);
                    span[2] = new Vector2d(WorldBounds.Max.x, Point.y);
                    span[3] = Point;
                    span[4] = new Vector2d(Point.x, WorldBounds.Max.y);
                    span[5] = WorldBounds.GetCorner(3);
                    break;
                case 1://NW
                    span[0] = WorldBounds.GetCorner(0);
                    span[1] = WorldBounds.GetCorner(1);
                    span[2] = WorldBounds.GetCorner(2);
                    span[3] = new Vector2d(Point.x, WorldBounds.Max.y);
                    span[4] = Point;
                    span[5] = new Vector2d(WorldBounds.Min.x, Point.y);
                    break;
                case 2://SW
                    span[0] = new Vector2d(WorldBounds.Min.x, Point.y);
                    span[1] = Point;
                    span[2] = new Vector2d(Point.x, WorldBounds.Min.y);
                    span[3] = WorldBounds.GetCorner(1);
                    span[4] = WorldBounds.GetCorner(2);
                    span[5] = WorldBounds.GetCorner(3);
                    break;
                case 3://SE
                    span[0] = WorldBounds.GetCorner(0);
                    span[1] = new Vector2d(Point.x, WorldBounds.Min.y);
                    span[2] = Point;
                    span[3] = new Vector2d(WorldBounds.Max.x, Point.y);
                    span[4] = WorldBounds.GetCorner(2);
                    span[5] = WorldBounds.GetCorner(3);
                    break;
                default: throw new ArgumentException(nameof(quadNr));
            }

            return quadrantCoverPoly;
        }
        #endregion
    }
}
