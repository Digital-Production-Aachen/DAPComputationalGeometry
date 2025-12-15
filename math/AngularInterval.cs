using System;

namespace g3
{
    public readonly record struct AngularInterval(double Start, double End)
    {
        public double Length
        {
            get
            {
                double span = End - Start;
                if (span < 0) span += 2 * Math.PI;
                return span;
            }
        }

        public static AngularInterval Normalized(double start, double end)
            => new(Normalize(start), Normalize(end));
        public static AngularInterval FullCircle => new AngularInterval(0, Math.PI * 2);

        private static double Normalize(double angle)
        {
            const double twoPi = 2 * Math.PI;
            angle %= twoPi;
            if (angle < 0) angle += twoPi;
            return angle;
        }

        public override string ToString()
            => $"[{Start:F3}, {End:F3}] (Length={Length:F3} rad)";
    }
}
