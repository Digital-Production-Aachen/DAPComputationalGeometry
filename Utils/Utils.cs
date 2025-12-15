using System;

namespace g3
{
    public static class Utils
    {
        public static double Rad2Deg(double phi) { return phi / Math.PI * 180; }
        public static double Deg2Rad(double phi) { return phi * Math.PI / 180; }
        public static double SnapToIntervalFloor(double value, double interval, int roundDigits)
        {
            value = Math.Round(value, roundDigits);
            var remainder = value % interval;
            var snapPoint = value - remainder;
            return value - snapPoint >= 0 ? snapPoint : snapPoint - interval;
        }

        public static double SnapToIntervalCeiling(double value, double interval, int roundDigits)
        {
            value = Math.Round(value, roundDigits);
            var remainder = value % interval;
            var snapPoint = value - remainder;
            return value - snapPoint > 0 ? snapPoint + interval : snapPoint;
        }

        /// <summary>
        /// Returns the desired fraction of available memory reported by the GC in bytes
        /// </summary>
        /// <param name="fraction"></param>
        /// <returns>size of fraction of available memory in bytes</returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static long AvailableMemoryFraction(double fraction)
        {
            if (fraction <= 0 || fraction > 1.0)
                throw new ArgumentOutOfRangeException(nameof(fraction));

            var gcInfo = GC.GetGCMemoryInfo();
            long total = gcInfo.TotalAvailableMemoryBytes;
            long max = (long)(total * fraction);
            return max;
        }
    }
}
