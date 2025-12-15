using g3;
using System;
using static FortuneVoronoi.VoronoiGraph;

namespace FortuneVoronoi
{
    internal abstract class VNode
    {
        private VNode _left;
        private VNode _right;

        public VNode Left
        {
            get { return _left; }
            set
            {
                _left = value;
                value.Parent = this;
            }
        }

        public VNode Right
        {
            get { return _right; }
            set
            {
                _right = value;
                value.Parent = this;
            }
        }

        public VNode Parent { get; set; }

        public void Replace(VNode childOld, VNode childNew)
        {
            if (Left == childOld)
                Left = childNew;
            else if (Right == childOld)
                Right = childNew;
            else
                throw new ArgumentException("Child not found!", nameof(childOld));
            childOld.Parent = null;
        }

        public static VDataNode FirstDataNode(VNode root)
        {
            var c = root;
            while (c.Left != null)
                c = c.Left;
            return (VDataNode)c;
        }

        public static VDataNode LeftDataNode(VDataNode current)
        {
            VNode c = current;

            //1. Up
            do
            {
                if (c.Parent == null)
                    return null;
                if (c.Parent.Left == c)
                {
                    c = c.Parent;
                }
                else
                {
                    c = c.Parent;
                    break;
                }
            } while (true);

            //2. One Left
            c = c.Left;

            //3. Down
            while (c.Right != null)
                c = c.Right;
            return (VDataNode)c; // Cast statt 'as' damit eine Exception kommt
        }

        public static VDataNode RightDataNode(VDataNode current)
        {
            VNode c = current;

            //1. Up
            do
            {
                if (c.Parent == null)
                    return null;
                if (c.Parent.Right == c)
                {
                    c = c.Parent;
                }
                else
                {
                    c = c.Parent;
                    break;
                }
            } while (true);

            //2. One Right
            c = c.Right;

            //3. Down
            while (c.Left != null)
                c = c.Left;
            return (VDataNode)c; // Cast statt 'as' damit eine Exception kommt
        }

        public static VEdgeNode EdgeToRightDataNode(VDataNode current)
        {
            VNode c = current;
            //1. Up
            do
            {
                if (c.Parent == null)
                    throw new Exception("No Left Leaf found!");
                if (c.Parent.Right == c)
                {
                    c = c.Parent;
                }
                else
                {
                    c = c.Parent;
                    break;
                }
            } while (true);
            return (VEdgeNode)c;
        }

        public static VDataNode FindDataNode(VNode root, double ys, double x)
        {
            var c = root;
            do
            {
                var node = c as VDataNode;
                if (node != null)
                    return node;

                c = ((VEdgeNode)c).Cut(ys, x) < 0 ? c.Left : c.Right;
            } while (true);
        }

        /// <summary>
        /// Will return the new root (unchanged except in start-up)
        /// </summary>
        public static VNode ProcessDataEvent(VDataEvent e, VNode root, VoronoiGraph vg, double ys, out VDataNode[] circleCheckList)
        {
            if (root == null)
            {
                root = new VDataNode(e.DataPoint);
                circleCheckList = [(VDataNode)root];
                return root;
            }
            //1. Find the node to be replaced
            VNode c = FindDataNode(root, ys, e.DataPoint.x);
            //2. Create the subtree (ONE Edge, but two VEdgeNodes)
            var ve = new Edge
            {
                LeftData = ((VDataNode)c).DataPoint,
                RightData = e.DataPoint,
                VVertexA = null,
                VVertexB = null
            };
            vg._mutableEdges.Add(ve);

            VNode subRoot;
            if (Math.Abs(ve.LeftData.y - ve.RightData.y) < 1e-10)
            {
                if (ve.LeftData.x < ve.RightData.x)
                {
                    subRoot = new VEdgeNode(ve, false)
                    {
                        Left = new VDataNode(ve.LeftData),
                        Right = new VDataNode(ve.RightData)
                    };
                }
                else
                {
                    subRoot = new VEdgeNode(ve, true)
                    {
                        Left = new VDataNode(ve.RightData),
                        Right = new VDataNode(ve.LeftData)
                    };
                }
                circleCheckList = new VDataNode[] { (VDataNode)subRoot.Left, (VDataNode)subRoot.Right };
            }
            else
            {
                subRoot = new VEdgeNode(ve, false)
                {
                    Left = new VDataNode(ve.LeftData),
                    Right = new VEdgeNode(ve, true)
                    {
                        Left = new VDataNode(ve.RightData),
                        Right = new VDataNode(ve.LeftData)
                    }
                };
                circleCheckList = new VDataNode[] { (VDataNode)subRoot.Left, (VDataNode)subRoot.Right.Left, (VDataNode)subRoot.Right.Right };
            }

            //3. Apply subtree
            if (c.Parent == null)
                return subRoot;
            c.Parent.Replace(c, subRoot);
            return root;
        }

        public static VNode ProcessCircleEvent(VCircleEvent e, VNode root, VoronoiGraph vg, out VDataNode[] circleCheckList)
        {
            VEdgeNode eo;
            var b = e.NodeN;
            var a = LeftDataNode(b);
            var c = RightDataNode(b);
            if (a == null || b.Parent == null || c == null || !a.DataPoint.Equals(e.NodeL.DataPoint) || !c.DataPoint.Equals(e.NodeR.DataPoint))
            {
                circleCheckList = new VDataNode[] { };
                return root; // Abbruch da sich der Graph verändert hat
            }
            var eu = (VEdgeNode)b.Parent;
            circleCheckList = new VDataNode[] { a, c };
            //1. Create the new Vertex
            var vNew = new Vector2d(e.Center.x, e.Center.y);
            //			VNew.x = Fortune.ParabolicCut(a.DataPoint.x,a.DataPoint.y,c.DataPoint.x,c.DataPoint.y,ys);
            //			VNew.y = (ys + a.DataPoint.y)/2 - 1/(2*(ys-a.DataPoint.y))*(VNew.x-a.DataPoint.x)*(VNew.x-a.DataPoint.x);
            vg._mutableVertices.Add(vNew);
            //2. Find out if a or c are in a distand part of the tree (the other is then b's sibling) and assign the new vertex
            if (eu.Left == b) // c is sibling
            {
                eo = EdgeToRightDataNode(a);

                // replace eu by eu's Right
                eu.Parent.Replace(eu, eu.Right);
            }
            else // a is sibling
            {
                eo = EdgeToRightDataNode(b);

                // replace eu by eu's Left
                eu.Parent.Replace(eu, eu.Left);
            }
            eu.Edge.AddVertex(vNew);
            //			///////////////////// uncertain
            //			if(eo==eu)
            //				return root;
            //			/////////////////////

            //complete & cleanup eo
            eo.Edge.AddVertex(vNew);
            //while(eo.Edge.VVertexB == Fortune.VVUnkown)
            //{
            //    eo.flipped = !eo.flipped;
            //    eo.Edge.AddVertex(Fortune.VVInfinite);
            //}
            //if(eo.flipped)
            //{
            //    Vector T = eo.Edge.LeftData;
            //    eo.Edge.LeftData = eo.Edge.RightData;
            //    eo.Edge.RightData = T;
            //}


            //2. Replace eo by new Edge
            var ve = new Edge
            {
                LeftData = a.DataPoint,
                RightData = c.DataPoint
            };
            ve.AddVertex(vNew);
            vg._mutableEdges.Add(ve);

            var ven = new VEdgeNode(ve, false)
            {
                Left = eo.Left,
                Right = eo.Right
            };
            if (eo.Parent == null)
                return ven;
            eo.Parent.Replace(eo, ven);
            return root;
        }
        public static VCircleEvent CircleCheckDataNode(VDataNode n, double ys)
        {
            var l = LeftDataNode(n);
            var r = RightDataNode(n);
            if (l == null || r == null || l.DataPoint == r.DataPoint || l.DataPoint == n.DataPoint || n.DataPoint == r.DataPoint)
                return null;
            if (ccw(l.DataPoint, n.DataPoint, r.DataPoint, false) <= 0)
                return null;
            var center = FortuneVoronoiAlgorithm.CircumCircleCenter(l.DataPoint, n.DataPoint, r.DataPoint);
            var vc = new VCircleEvent
            {
                NodeN = n,
                NodeL = l,
                NodeR = r,
                Center = center,
                Valid = true
            };
            if (vc.Y > ys || Math.Abs(vc.Y - ys) < 1e-10)
                return vc;
            return null;
        }

        private static int ccw(Vector2d p0, Vector2d p1, Vector2d p2, bool plusOneOnZeroDegrees)
        {
            var d1 = p1 - p0;
            var d2 = p2 - p0;

            if (d1.x * d2.y > d1.y * d2.x)
                return +1;

            if (d1.x * d2.y < d1.y * d2.x)
                return -1;

            if ((d1.x * d2.x < 0) || (d1.y * d2.y < 0))
                return -1;

            if (d1.x * d1.x + d1.y * d1.y < d2.x * d2.x + d2.y * d2.y && plusOneOnZeroDegrees)
                return +1;

            return 0;
        }

        public static void CleanUpTree(VEdgeNode ve)
        {
            if (ve == null)
                return;

            while (!ve.Edge.VVertexB.HasValue)
            {
                ve.Edge.AddVertex(FortuneVoronoiAlgorithm.VvInfinite);
            }
            if (ve.Flipped)
            {
                var t = ve.Edge.LeftData;
                ve.Edge.LeftData = ve.Edge.RightData;
                ve.Edge.RightData = t;
            }
            ve.Edge.Done = true;
            CleanUpTree(ve.Left as VEdgeNode);
            CleanUpTree(ve.Right as VEdgeNode);
        }
    }

    internal class VDataNode : VNode
    {
        public VDataNode(Vector2d dp)
        {
            DataPoint = dp;
        }
        public Vector2d DataPoint;
    }

    internal class VEdgeNode : VNode
    {
        public VEdgeNode(Edge e, bool flipped)
        {
            Edge = e;
            Flipped = flipped;
        }
        public readonly Edge Edge;
        public readonly bool Flipped;
        public double Cut(double ys, double x)
        {
            if (!Flipped)
                return Math.Round(x - FortuneVoronoiAlgorithm.ParabolicCut(Edge.LeftData.x, Edge.LeftData.y, Edge.RightData.x, Edge.RightData.y, ys), 10);
            return Math.Round(x - FortuneVoronoiAlgorithm.ParabolicCut(Edge.RightData.x, Edge.RightData.y, Edge.LeftData.x, Edge.LeftData.y, ys), 10);
        }
    }
}
