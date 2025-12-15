using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace g3
{
    public class ThreadSafeBitArray
    {
        private const int size_t = 64;
        private const int shift_t = 6;//64 = 2^6

        private int _length;
        private ulong[] _arr;
        public ThreadSafeBitArray(int length)
        {
            _length = length;
            _arr = new ulong[ToUnderlineLength(length)];
            SetUnusedBitsTrue();
        }

        public ThreadSafeBitArray(ulong[] bits, int length)
        {
            _length = length;
            _arr = bits;
            SetUnusedBitsTrue();
        }

        private void SetUnusedBitsTrue()
        {
            // unused bits are always set for simpler SearchClosestUnset
            int rest = _length % size_t;
            if (rest > 0) { _arr[^1] |= ~0ul << rest; }
        }

        private int ToUnderlineLength(int length)
        {
            int underlineLength = length / size_t;

            if (length % size_t != 0)
            {
                underlineLength++;
            }

            return underlineLength;
        }

        public int Length => _length;

        public int PopulationCount
        {
            get
            {
                int popCount = 0;
                for (int i = 0; i < _arr.Length; i++)
                {
                    popCount += BitOperations.PopCount(_arr[i]);
                }
                //this behavior depends on that unused bits are always SET
                int rest = _length % size_t;
                if(rest > 0) { popCount -= size_t - rest; }
                return popCount;
            }
        }

        public int SearchClosestUnset(int start)
        {
            if (!this[start]) { return start; } //will do range check and throw out of bounds
            int uLongIdx = start >> shift_t;            
            ulong target = _arr[uLongIdx];
            if (target == ~0ul) //all bits set?
            {
                // search the closest ulong
                int left = uLongIdx - 1;
                int right = uLongIdx + 1;
                while (left >= 0 || right < _arr.Length)
                {
                    if (right < _arr.Length)
                    {
                        target = _arr[right];
                        if (target < ~0ul)
                        {
                            int result = (right * size_t) + BitOperations.TrailingZeroCount(~target);
                            //Debug.Assert(this[result] == false); //these asserts fail multi threaded (on concurrent edits)
                            return result;
                        }
                    }
                    if (left >= 0)
                    {
                        target = _arr[left];
                        if (target < ~0ul)
                        {
                            int result = (left * size_t) + size_t - 1 - BitOperations.LeadingZeroCount(~target);
                            //Debug.Assert(this[result] == false);
                            return result;
                        }
                    }
                    left--;
                    right++;
                }
                // all set
                return -1;
            }
            else
            {
                start = start % size_t;
                int nextRight = BitOperations.TrailingZeroCount(~target >> start);                
                int idxLeft = start - BitOperations.LeadingZeroCount(~target << size_t - start) - 1;
                if (nextRight < size_t && (idxLeft < 0 || nextRight <= Math.Abs(start - idxLeft)))
                {
                    int result = size_t * uLongIdx + start + nextRight;
                    //Debug.Assert(this[result] == false);
                    return result;
                }
                else
                {
                    int result = size_t * uLongIdx + idxLeft;
                    //Debug.Assert(this[result] == false);
                    return result;
                }
            }
        }

        public void SetAll(bool value)
        {
            if (value)
                Array.Fill(_arr, ulong.MaxValue);
            else
            {
                Array.Fill(_arr, 0ul);
                SetUnusedBitsTrue();
            }
        }

        public bool this[int index]
        {
            get
            {
                if (index >= Length) throw new ArgumentOutOfRangeException();
                ulong mask = 1ul << (index % size_t);
                return (_arr[index >> shift_t] & mask) != 0;
            }
            set
            {
                if (index >= Length) throw new IndexOutOfRangeException();
                ulong prevValue;
                ref ulong target = ref _arr[index >> shift_t];
                ulong mask = 1ul << (index % size_t);
                if (value)
                {
                    do { prevValue = target; }
                    while (Interlocked.CompareExchange(ref target, prevValue | mask, prevValue) != prevValue);
                }
                else
                {
                    do { prevValue = target; }
                    while (Interlocked.CompareExchange(ref target, prevValue & ~mask, prevValue) != prevValue);
                }
            }
        }

        /// <summary>
        /// Tries to set the bit at index to true in a thread safe way.
        /// Returns falls if the bit has already been set (may have happened concurrently).
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public bool TrySet(int index)
        {
            if(index >= Length) throw new IndexOutOfRangeException();
            ulong prevValue;
            ref ulong target = ref _arr[index >> shift_t];
            ulong mask = 1ul << (index % size_t);
            do
            {
                prevValue = target;
                if ((prevValue & mask) != 0) { return false; }
            }
            while (Interlocked.CompareExchange(ref target, prevValue | mask, prevValue) != prevValue);
            return true;
        }

        public override string ToString()
        {
            return string.Create(_length, this, (stringSpan, bitArray) =>
            {
                for (int i = 0; i < _length; i++)
                    stringSpan[i] = bitArray[i] ? '1' : '0';
            });
        }

        public static string ToBitString(ulong value)
        {
            Span<char> c = stackalloc char[64];
            for (int i = 0; i < 64; i++)
                c[63 - i] = ((value >> i) & 1) == 1 ? '1' : '0';
            return new string(c);
        }
    }
}
