using g3;
using System;

namespace FortuneVoronoi
{
    internal abstract class VEvent : IComparable<VEvent>
    {
        public abstract double Y { get; }
        protected abstract double X { get; }

        #region IComparable Members
        public int CompareTo(VEvent evt)
        {
            var i = Y.CompareTo(evt.Y);
            if (i != 0)
                return i;
            return X.CompareTo(evt.X);
        }
        #endregion
    }

    internal class VDataEvent : VEvent
    {
        public Vector2d DataPoint;

        public VDataEvent(Vector2d dp)
        {
            DataPoint = dp;
        }

        public override double Y
        {
            get
            {
                return DataPoint.y;
            }
        }

        protected override double X
        {
            get
            {
                return DataPoint.x;
            }
        }

    }

    internal class VCircleEvent : VEvent
    {
        public VDataNode NodeN, NodeL, NodeR;
        public Vector2d Center;

        public override double Y
        {
            get { return Math.Round(Center.y + NodeN.DataPoint.Distance(Center), 10); }
        }

        protected override double X
        {
            get
            {
                return Center.x;
            }
        }

        public bool Valid = true;
    }
}
