using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace g3
{
    public class InterlockedHelpers
    {
        public static int Max(ref int location1, int max)
        {
            int curValue = location1;
            if (curValue >= max) return curValue;
            while ((curValue = Interlocked.CompareExchange(ref location1, max, curValue)) < max) { };
            return Math.Max(curValue, max);
        }

        public static double Max(ref double location1, double max)
        {
            double curValue = location1;
            if (curValue >= max) return curValue;
            while ((curValue = Interlocked.CompareExchange(ref location1, max, curValue)) < max) { };
            return Math.Max(curValue, max);
        }

        public static int Min(ref int location1, int min)
        {
            int curValue = location1;
            if (curValue <= min) return curValue;
            while ((curValue = Interlocked.CompareExchange(ref location1, min, curValue)) > min) { };
            return Math.Min(curValue, min);
        }

        public static double Min(ref double location1, double min)
        {
            double curValue = location1;
            if (curValue <= min) return curValue;
            while ((curValue = Interlocked.CompareExchange(ref location1, min, curValue)) > min) { };
            return Math.Min(curValue, min);
        }

        public static int IncrementAbs(ref int location1)
        {
            int curValue;
            int actualValue = location1;
            int newValue;
            do
            {
                curValue = actualValue;
                newValue = curValue >= 0 ? curValue + 1 : curValue - 1;
                actualValue = Interlocked.CompareExchange(ref location1, newValue, curValue);
            }
            while (actualValue != curValue);
            return newValue;
        }

        public static int DecrementAbs(ref int location1)
        {
            int curValue;
            int actualValue = location1;
            int newValue;
            do
            {
                curValue = actualValue;
                newValue = curValue < 0 ? curValue + 1 : curValue - 1;
                actualValue = Interlocked.CompareExchange(ref location1, newValue, curValue);
            }
            while (actualValue != curValue);
            return newValue;
        }

        public static int SetSignBitNegative(ref int location1)
        {
            int curValue;
            int actualValue = location1;
            int newValue;
            do
            {
                curValue = actualValue;
                if (curValue < 0) break;
                newValue = -curValue;
                actualValue = Interlocked.CompareExchange(ref location1, newValue, curValue);
            }
            while (actualValue != curValue);
            return curValue;
        }
    }
}
