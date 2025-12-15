using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace g3
{
    public struct BitArray256
    {
        private BitVector256 first;
        private int _length;
        private BitVector256[] rest;

        public int Length => _length;

        public BitArray256()
        {
            _length = BitVector256.Length;
        }

        public BitArray256(int length)
        {
            _length = length;
            AllocArray();
        }

        public BitArray256(BitArray256 toCopy, int offset, int paddingLength)
        {
            _length = Math.Max(toCopy.Length + offset, paddingLength);
            if (offset == 0 && _length < BitVector256.Length)
            {
                first = toCopy.first;
                return;
            }
            int ulongSrcCount = (toCopy._length / 64) + 1;
            int ulongOffset = offset / 64;
            var result = new ulong[((_length / 256) + 1) * 4];
            var bitOffset = offset % 64;

            ulong prevLower = toCopy.GetULong(ulongOffset) << bitOffset;
            for (int i = 0; i < ulongSrcCount; i++)
            {
                var iSource = i + ulongOffset + 1;
                ulong current = toCopy.GetULong(iSource);
                ulong currentUpper = current >> bitOffset;
                int reversedIdx = (3 - (i % 4)) + ((i / 4) * 4);
                result[reversedIdx] |= currentUpper;
                result[reversedIdx] |= prevLower;
                prevLower = current << bitOffset;
            }

            var span = result.AsSpan();
            first = new BitVector256(span);

            if (AllocArray())
            {
                for (int i = 0; i < rest.Length; i++)
                {
                    rest[i] = new BitVector256(span.Slice(4 * i + 4));
                }
            }
        }

        private ulong GetULong(int i)
        {
            BitVector256 vec256Source;
            if (i < 0 || i > Length / 256)
                return 0;//pad with zeros
            else if (i < 4)
                vec256Source = first;
            else
                vec256Source = rest[(i / 4) - 1];
            return vec256Source.GetULong(i % 4);
        }

        private bool AllocArray()
        {
            if (_length > BitVector256.Length)
            {
                var arrayCount = _length / BitVector256.Length;
                rest = new BitVector256[arrayCount];
                return true;
            }
            return false;
        }

        private BitArray256(BitVector256 value, int length)
        {
            _length = length;
            first = value;
        }

        public bool this[int index]
        {
            get
            {
                if (index < BitVector256.Length)
                {
                    return first[index];
                }
                else
                {
                    int diff = index - BitVector256.Length;
                    return rest[diff / BitVector256.Length][index % BitVector256.Length];
                }
            }
            set
            {
                if (index < BitVector256.Length)
                {
                    first[index] = value;
                }
                else
                {
                    int diff = index - BitVector256.Length;
                    rest[diff / BitVector256.Length][index % BitVector256.Length] = value;
                }
            }
        }

        public bool Any()
        {
            if (!first.Any())
                return false;

            for (int i = 0; i < rest?.Length; i++)
            {
                if (!rest[i].Any())
                    return false;
            }
            return true;
        }

        public bool All()
        {
            return PopulationCount == _length;
        }

        public int PopulationCountComplement => _length - PopulationCount;

        public int PopulationCount
        {
            get
            {
                int popCount = 0;
                int ulongCount = (_length / 64) + 1;
                for (int i = 0; i < ulongCount; i++)
                {
                    ulong current = GetULong(i);
                    popCount += BitOperations.PopCount(current);
                }
                return popCount;
            }
        }

        public int TrailingZeroCount
        {
            get
            {
                int firstSetBit = 0;
                int ulongCount = (_length / 64) + 1;
                for (int i = 0; i < ulongCount; i++)
                {
                    ulong current = GetULong(i);
                    firstSetBit += BitOperations.TrailingZeroCount(current);
                    if (current != 0) break;
                }
                return Math.Min(firstSetBit, _length);
            }
        }

        private const ulong ulongAllSet = 0xffffffffffffffffL;

        public static BitArray256 Ones(int length)
        {
            var result = new ulong[((length / 256) + 1) * 4];

            int fullUlongCount = length / 64;
            for (int i = 0; i < fullUlongCount; i++)
            {
                result[i] = ulongAllSet;
            }

            ulong lastULong = ulongAllSet << (length % 64);
            result[fullUlongCount + 1] = lastULong;

            var span = result.AsSpan();
            var newOnes = new BitArray256(length);
            newOnes.first = new BitVector256(span);

            for (int i = 0; i < newOnes.rest?.Length; i++)
            {
                newOnes.rest[i] = new BitVector256(span.Slice(4 * i + 4));
            }
            return newOnes;
        }

        public static bool operator ==(BitArray256 left, BitArray256 right)
        {
            if (left.Length != right.Length)
                return false;
            if (left.first != right.first)
                return false;

            for (int i = 0; i < left.Length; i++)
            {
                if (left.rest[i] != right.rest[i])
                    return false;
            }
            return true;
        }

        public static bool operator !=(BitArray256 left, BitArray256 right)
        {
            return !(left == right);
        }

        public static BitArray256 operator &(BitArray256 left, BitArray256 right)
        {
            if (left._length != right._length)
                throw new ArgumentException("lengths don't match");
            var result = new BitArray256(left.first & right.first, left._length);
            if (left.Length <= BitVector256.Length)
                return result;

            result.AllocArray();
            for (int i = 0; i < left.rest.Length; i++)
            {
                result.rest[i] = left.rest[i] & right.rest[i];
            }
            return result;
        }

        public static BitArray256 operator |(BitArray256 left, BitArray256 right)
        {
            if (left._length != right._length)
                throw new ArgumentException("lengths don't match");
            var result = new BitArray256(left.first | right.first, left._length);
            if (left.Length <= BitVector256.Length)
                return result;

            result.AllocArray();
            for (int i = 0; i < left.rest.Length; i++)
            {
                result.rest[i] = left.rest[i] | right.rest[i];
            }
            return result;
        }
        public static BitArray256 operator ~(BitArray256 vec)
        {
            var result = new BitArray256(~vec.first, vec._length);
            if (vec.Length <= BitVector256.Length)
                return result;

            result.AllocArray();
            for (int i = 0; i < vec.rest.Length; i++)
            {
                result.rest[i] = ~vec.rest[i];
            }
            return result;
        }

        public override bool Equals(object obj)
        {
            return obj is BitArray256 arr && arr == this;
        }

        public override int GetHashCode()
        {
            return first.GetHashCode();
        }
    }

    public struct BitVector256
    {
        private Vector256<ulong> bits;

        private static readonly Vector256<ulong> Zero256 = Vector256<ulong>.Zero;

        public BitVector256(Vector256<ulong> bits)
        {
            this.bits = bits;
        }

        public BitVector256(ReadOnlySpan<ulong> bits)
        {
            this.bits = Vector256.Create(bits);
        }

        public static readonly BitVector256 Zero = new BitVector256(Zero256);
        public static readonly BitVector256 AllBitsSet = new BitVector256(Vector256<ulong>.AllBitsSet);
        private const ulong ulongAllSet = 0xffffffffffffffffL;

        public static BitVector256 OnesFrom(int oneStartIdx)
        {
            if(oneStartIdx == 256) { return Zero; }
            Span<ulong> bits = stackalloc ulong[4];
            var idx = 3 - (oneStartIdx / 64);
            oneStartIdx %= 64;
            ulong mask = ulongAllSet << oneStartIdx;
            bits[idx--] = mask;
            for(; idx >= 0; idx--) { bits[idx] = ulongAllSet; }
            return new BitVector256(bits);
        }

        /// <summary>
        /// Get or set the bit at the given index. For faster getting of multiple
        /// bits, use <see cref="GetBits(BitVector256)"/>. For faster setting of single
        /// bits, use <see cref="SetBits(BitVector256)"/> or <see cref="UnsetBits(BitVector256)"/>. For
        /// faster setting of multiple bits, use <see cref="SetBits(BitVector256)"/> or
        /// <see cref="UnsetBits(BitVector256)"/>.
        /// </summary>
        /// 
        /// <param name="index">
        /// Index of the bit to get or set
        /// </param>
        public bool this[int index]
        {
            get
            {
                var ulongIdx = index / 64;
                index %= 64;
                ulong mask = 1ul << index;
                return (GetULong(ulongIdx) & mask) == mask;
            }
            set
            {
                var mask = MakeMask(index);
                if (value)
                {
                    bits |= mask;
                }
                else
                {
                    bits &= ~mask;
                }
            }
        }

        internal ulong GetULong(int index) => bits[3 - index];

        private static readonly Vector256<ulong> One0 = Vector256.Create(new ulong[] { 0, 0, 0, 1 });
        private static readonly Vector256<ulong> One1 = Vector256.Create(new ulong[] { 0, 0, 1, 0 });
        private static readonly Vector256<ulong> One2 = Vector256.Create(new ulong[] { 0, 1, 0, 0 });
        private static readonly Vector256<ulong> One3 = Vector256.Create(new ulong[] { 1, 0, 0, 0 });
        private Vector256<ulong> MakeMask(int index)
        {
            var shift = index % 64;
            var idx = index / 64;

            switch (idx)
            {
                case 0:
                    return Vector256.ShiftLeft(One0, shift);
                case 1:
                    return Vector256.ShiftLeft(One1, shift);
                case 2:
                    return Vector256.ShiftLeft(One2, shift);
                case 3:
                    return Vector256.ShiftLeft(One3, shift);
                default:
                    throw new IndexOutOfRangeException(index.ToString());
            }
        }

        /// <summary>
        /// Returns the number of bits set to 1
        /// </summary>
        /// <returns></returns>
        public int PopulationCount
        {
            get
            {
                int popCount = BitOperations.PopCount(bits[0]);
                popCount += BitOperations.PopCount(bits[1]);
                popCount += BitOperations.PopCount(bits[2]);
                popCount += BitOperations.PopCount(bits[3]);
                return popCount;
            }
        }

        public int PopulationCountComplement => Length - PopulationCount;

        /// <summary>
        /// Get the length of the array
        /// </summary>
        /// 
        /// <value>
        /// The length of the array. Always 256.
        /// </value>
        public const int Length = 256;

        /// <summary>
        /// Returns if any bits are set
        /// </summary>
        /// <returns></returns>
        public bool Any()
        {
            return bits != Zero256;
        }

        /// <summary>
        /// Returns if all bits are set
        /// </summary>
        /// <returns></returns>
        public bool All()
        {
            return bits == Vector256<ulong>.AllBitsSet;
        }

        public int TrailingZeroCount
        {
            get
            {
                for (int i = 3; i >= 0; i--)
                {
                    ulong current = bits[i];
                    if (current != 0) { return BitOperations.TrailingZeroCount(current) + (3 - i) * 64; }
                }
                return Length;
            }
        }

        public int LeadingZeroCount
        {
            get
            {
                for (int i = 0; i < 4; i++)
                {
                    ulong current = bits[i];
                    if (current != 0) { return BitOperations.LeadingZeroCount(current) + i * 64; }
                }
                return Length;
            }
        }

        /// <summary>
        /// Get all the bits that match a mask
        /// </summary>
        /// 
        /// <param name="mask">
        /// Mask of bits to get
        /// </param>
        /// 
        /// <returns>
        /// The bits that match the given mask
        /// </returns>
        public BitVector256 GetBits(BitVector256 mask)
        {
            return new BitVector256(bits & mask.bits);
        }

        /// <summary>
        /// Set all the bits that match a mask to 1
        /// </summary>
        /// 
        /// <param name="mask">
        /// Mask of bits to set
        /// </param>
        public void SetBits(BitVector256 mask)
        {
            bits |= mask.bits;
        }

        /// <summary>
        /// Set all the bits that match a mask to 0
        /// </summary>
        /// 
        /// <param name="mask">
        /// Mask of bits to unset
        /// </param>
        public void UnsetBits(BitVector256 mask)
        {
            bits &= ~mask.bits;
        }

        public override bool Equals(object obj)
        {
            return obj is BitVector256 vec && bits == vec.bits;
        }

        public bool Equals(BitVector256 arr)
        {
            return bits == arr.bits;
        }

        public override int GetHashCode()
        {
            return bits.GetHashCode();
        }

        public override string ToString()
        {
            const string header = "BitArray256{";
            const int headerLen = 12; // must be header.Length
            char[] chars = new char[headerLen + 256 + 1];
            int i = 0;
            for (; i < headerLen; ++i)
            {
                chars[i] = header[i];
            }
            int j = 0;
            for (; i < chars.Length - 1; ++i)
            {
                for (ulong num = 1ul << 63; num > 0; num >>= 1, ++i)
                {
                    chars[i] = (bits[j] & num) != 0 ? '1' : '0';
                }
                ++j;
                --i;
            }
            chars[i] = '}';
            return new string(chars);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BitVector256 left, BitVector256 right)
        {
            return left.bits == right.bits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BitVector256 left, BitVector256 right)
        {
            return left.bits != right.bits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BitVector256 operator &(BitVector256 left, BitVector256 right)
        {
            return new BitVector256(left.bits & right.bits);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BitVector256 operator |(BitVector256 left, BitVector256 right)
        {
            return new BitVector256(left.bits | right.bits);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BitVector256 operator ^(BitVector256 left, BitVector256 right)
        {
            return new BitVector256(left.bits ^ right.bits);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BitVector256 operator ~(BitVector256 vec)
        {
            return new BitVector256(~vec.bits);
        }
    }
}
