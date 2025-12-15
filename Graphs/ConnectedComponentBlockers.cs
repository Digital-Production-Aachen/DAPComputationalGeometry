using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace g3
{
    /// <summary>
    /// ConnectedComponentBlockers keeps track of marked combinations (sets and subsets) of nodes of connected components of a graph.
    /// Supports marking combinations of nodes as a "blocker", querying if a set is marked (contains a blocker as subset)
    /// and union operations when connected components are merged. Also supports detecting cycles in the added blocker sets that cause a deadlock.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class ConnectedComponentBlockers
    {
        private struct BinaryIntegerBitSet<TInt> : IBitSet<BinaryIntegerBitSet<TInt>>
            where TInt : IBinaryInteger<TInt>
        {
            private TInt v;

            public BinaryIntegerBitSet() { v = TInt.Zero; }
            public BinaryIntegerBitSet(TInt v) { this.v = v; }
            public static int Capacity => (typeof(TInt) == typeof(BigInteger)) ? int.MaxValue : Unsafe.SizeOf<TInt>() * 8;
            public void SetBit(int bitIndex) => v |= (TInt.One << bitIndex);
            public void ClearBit(int bitIndex) => v &= ~(TInt.One << bitIndex);
            public void OrWith(in BinaryIntegerBitSet<TInt> other) => v |= other.v;
            public bool IsSubsetOf(in BinaryIntegerBitSet<TInt> other) => (v & ~other.v) == TInt.Zero;
            public int PopCount() => int.CreateSaturating(TInt.PopCount(v));
            public int TrailingZeroCount() => int.CreateSaturating(TInt.TrailingZeroCount(v));
            public BinaryIntegerBitSet<TInt> Clone() => this;
            public Span<ulong> AsSpan() => MemoryMarshal.Cast<BinaryIntegerBitSet<TInt>, ulong>(MemoryMarshal.CreateSpan(ref this, 1));
        }

        private interface IBitSet<TSelf> where TSelf : IBitSet<TSelf>, new()
        {
            static abstract int Capacity { get; }

            void SetBit(int bitIndex);
            void ClearBit(int bitIndex);
            void OrWith(in TSelf other);
            bool IsSubsetOf(in TSelf other);

            int PopCount();
            int TrailingZeroCount();

            TSelf Clone();
            Span<ulong> AsSpan();
        }

        private interface IBitSetStorage<TStorage>
           where TStorage : unmanaged, IBitSetStorage<TStorage>
        {
            Span<ulong> AsSpan();
            TStorage Clone();
        }

        private struct VectorBitSet<TStorage> : IBitSet<VectorBitSet<TStorage>>
            where TStorage : unmanaged, IBitSetStorage<TStorage>
        {
            private TStorage storage;

            public VectorBitSet() { storage = new TStorage(); }
            public VectorBitSet(TStorage storage) { this.storage = storage; }

            public static int Capacity => Unsafe.SizeOf<TStorage>() * 8;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetBit(int bitIndex)
            {
                var span = storage.AsSpan();
                if ((uint)bitIndex >= (uint)(span.Length * 64)) throw new ArgumentOutOfRangeException(nameof(bitIndex));
                int ulongIdx = bitIndex >> 6;
                int offset = bitIndex & 63;
                span[ulongIdx] |= (1UL << offset);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ClearBit(int bitIndex)
            {
                var span = storage.AsSpan();
                if ((uint)bitIndex >= (uint)(span.Length * 64)) throw new ArgumentOutOfRangeException(nameof(bitIndex));
                int ulongIdx = bitIndex >> 6;
                int offset = bitIndex & 63;
                span[ulongIdx] &= ~(1UL << offset);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OrWith(in VectorBitSet<TStorage> other)
            {
                Span<ulong> a = storage.AsSpan();
                ReadOnlySpan<ulong> b = other.storage.AsSpan();
                int laneCount = Vector<ulong>.Count;
                int n = a.Length;
                int vcount = n / laneCount;

                // vectorized portion
                var va = MemoryMarshal.Cast<ulong, Vector<ulong>>(a);
                var vb = MemoryMarshal.Cast<ulong, Vector<ulong>>(b);
                for (int i = 0; i < vcount; ++i)
                    va[i] = Vector.BitwiseOr(va[i], vb[i]);

                // tail scalars
                for (int i = vcount * laneCount; i < n; ++i)
                    a[i] |= b[i];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsSubsetOf(in VectorBitSet<TStorage> other)
            {
                ReadOnlySpan<ulong> a = storage.AsSpan();
                ReadOnlySpan<ulong> b = other.storage.AsSpan();
                int laneCount = Vector<ulong>.Count;
                int n = a.Length;
                int vcount = n / laneCount;

                var va = MemoryMarshal.Cast<ulong, Vector<ulong>>(a);
                var vb = MemoryMarshal.Cast<ulong, Vector<ulong>>(b);
                var zeroVec = Vector<ulong>.Zero;

                for (int i = 0; i < vcount; ++i)
                {
                    var tmp = Vector.AndNot(va[i], vb[i]);       // a & ~b
                    if (!Vector.EqualsAll(tmp, zeroVec)) return false;
                }

                // tail scalars
                for (int i = vcount * laneCount; i < n; ++i)
                {
                    if ((a[i] & ~b[i]) != 0UL) return false;
                }

                return true;
            }

            public int PopCount()
            {
                var span = storage.AsSpan();
                int sum = 0;
                for (int i = 0; i < span.Length; ++i) sum += BitOperations.PopCount(span[i]);
                return sum;
            }

            public int TrailingZeroCount()
            {
                var span = storage.AsSpan();
                ulong part = span[0];
                int result = 0;
                for (int i = 1; (part == 0) && (i < span.Length); i++)
                {
                    part = span[i];
                    result += (sizeof(ulong) * 8);
                }
                result += (int)ulong.TrailingZeroCount(part);
                return result;
            }

            public VectorBitSet<TStorage> Clone() => this;
            public Span<ulong> AsSpan() => storage.AsSpan();
        }

        [InlineArray(Length)]
        private struct BitSet256 : IBitSetStorage<BitSet256>
        {
            public const int Length = 4;
            private ulong element0;

            public Span<ulong> AsSpan() => MemoryMarshal.CreateSpan(ref Unsafe.As<BitSet256, ulong>(ref this), Length);

            public BitSet256 Clone() => this;
        }

        [InlineArray(Length)]
        private struct BitSet512 : IBitSetStorage<BitSet512>
        {
            public const int Length = 8;
            private ulong element0;

            public Span<ulong> AsSpan() => MemoryMarshal.CreateSpan(ref Unsafe.As<BitSet512, ulong>(ref this), Length);

            public BitSet512 Clone() => this;
        }

        [InlineArray(Length)]
        private struct BitSet1024 : IBitSetStorage<BitSet1024>
        {
            public const int Length = 16;
            private ulong element0;

            public Span<ulong> AsSpan() => MemoryMarshal.CreateSpan(ref Unsafe.As<BitSet1024, ulong>(ref this), Length);

            public BitSet1024 Clone() => this;
        }

        [InlineArray(Length)]
        private struct BitSet2048 : IBitSetStorage<BitSet2048>
        {
            public const int Length = 32;
            private ulong element0;

            public Span<ulong> AsSpan() => MemoryMarshal.CreateSpan(ref Unsafe.As<BitSet2048, ulong>(ref this), Length);

            public BitSet2048 Clone() => this;
        }

        private interface IBitSetList
        {
            int Count { get; }
            int MaxCount { get; }

            bool AddBlocker(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit);
            IBitSetList CopyWiden(int count);
            IEnumerable<int[]> ExtractBlockers(Dictionary<int, int> idxToBit);
            bool IsKnownBlocking(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit);
        }


        private sealed class BitSetList<T> : IBitSetList where T : IBitSet<T>, new()
        {
            private List<T> blockingSubsets = new();
            public int Count => blockingSubsets.Count;
            public int MaxCount => T.Capacity;

            public IBitSetList CopyWiden(int count)
            {
                if (count <= MaxCount) throw new ArgumentOutOfRangeException(nameof(count), "count must be greater than current capacity");
                if (count <= 128)
                {
                    BitSetList<BinaryIntegerBitSet<UInt128>> list128 = new();
                    list128.blockingSubsets.EnsureCapacity(blockingSubsets.Count);
                    foreach (T blocker in blockingSubsets)
                    {
                        UInt128 uint128 = UInt128.CreateSaturating(blocker.AsSpan()[0]);
                        list128.blockingSubsets.Add(new BinaryIntegerBitSet<UInt128>(uint128));
                    }
                    return list128;
                }
                else if (count < 2048)
                {
                    return count switch
                    {
                        < 256 => CopyToBitStorage<BitSet256>(),
                        < 512 => CopyToBitStorage<BitSet512>(),
                        < 1024 => CopyToBitStorage<BitSet1024>(),
                        _ => CopyToBitStorage<BitSet2048>(),
                    };
                }
                else
                {
                    BitSetList<BinaryIntegerBitSet<BigInteger>> listBig = new();
                    listBig.blockingSubsets.EnsureCapacity(blockingSubsets.Count);
                    foreach (T blocker in blockingSubsets)
                    {
                        BigInteger bigInt = new BigInteger(MemoryMarshal.Cast<ulong, byte>(blocker.AsSpan()));
                        listBig.blockingSubsets.Add(new BinaryIntegerBitSet<BigInteger>(bigInt));
                    }
                    return listBig;
                }
            }

            private IBitSetList CopyToBitStorage<TStorage>() where TStorage : unmanaged, IBitSetStorage<TStorage>
            {
                BitSetList<VectorBitSet<TStorage>> listBitStorage = new();
                listBitStorage.blockingSubsets.EnsureCapacity(blockingSubsets.Count);
                foreach (T blocker in blockingSubsets)
                {
                    TStorage storage = new TStorage();
                    blocker.AsSpan().CopyTo(storage.AsSpan());
                    listBitStorage.blockingSubsets.Add(new VectorBitSet<TStorage>(storage));
                }
                return listBitStorage;
            }

            public bool IsKnownBlocking(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit)
            {
                T mask = MakeMaskRead(blockingCombination, idxToBit, out bool overflow);
                if (overflow) return false;
                return AnySubsetOfMask(mask);
            }

            public IEnumerable<int[]> ExtractBlockers(Dictionary<int, int> idxToBit)
            {
                Debug.Assert(idxToBit.Values.ToArray().IsSortedBy(x => x), 
                    "we rely on the implementation detail that dictionary kvps are enumerated in insertion order, " +
                    "which might not be true in future versions of DOTNET (I think it will, if you see this assert I was wrong)");
                int[] bitToIdx = idxToBit.Keys.ToArray();
                foreach (T blocker in blockingSubsets)
                {
                    int count = blocker.PopCount();
                    int[] ints = new int[count];
                    T curMask = blocker.Clone();
                    for (int i = 0; i < count; i++)
                    {
                        int idx = curMask.TrailingZeroCount();
                        ints[i] = bitToIdx[idx];
                        //flip the last bit currently set
                        curMask.ClearBit(idx);
                    }
                    yield return ints;
                }
            }

            public bool AddBlocker(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit)
            {
                T mask = MakeMaskAdd(blockingCombination, idxToBit);
                if (AnySubsetOfMask(mask))
                    return false;
                //deduplicate entries based on subset
                blockingSubsets.RemoveAll(blocker => mask.IsSubsetOf(blocker));
                blockingSubsets.Add(mask);
                Debug.Assert(IsKnownBlocking(blockingCombination, idxToBit));
                return true;
            }

            private T MakeMaskRead(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit, out bool overflow)
            {
                T mask = new();
                foreach (int idx in blockingCombination)
                {
                    bool found = idxToBit.TryGetValue(idx, out int bitIdx);
                    if (!found) continue;
                    if (bitIdx >= T.Capacity)
                    {
                        overflow = true;
                        return mask;
                    }
                    mask.SetBit(bitIdx);
                }
                overflow = false;
                return mask;
            }

            private T MakeMaskAdd(IEnumerable<int> blockingCombination, Dictionary<int, int> idxToBit)
            {
                T mask = new();
                foreach (int idx in blockingCombination)
                {
                    bool found = idxToBit.TryGetValue(idx, out int bitIdx);
                    if (!found) idxToBit.Add(idx, idxToBit.Count);
                    mask.SetBit(bitIdx);
                }

                return mask;
            }

            private bool AnySubsetOfMask(T mask)
            {
                foreach (T blocker in blockingSubsets)
                {
                    if (blocker.IsSubsetOf(mask)) return true;
                }
                return false;
            }
        }

        /// <summary>
        /// Mapping of indices of instances contained in this connected component.
        /// The order is determined by unions of the connected components, not job add order.
        /// </summary>
        private readonly Dictionary<int, int> idxToBit = new();

        private readonly Dictionary<int, IBitSetList> blockingSets = new();
        public int Count => idxToBit.Count;
        public int BlockerSetsCount => blockingSets.Sum(x => x.Value.Count);

        public bool Contains(int idx) => idxToBit.ContainsKey(idx);

        public IEnumerable<(int movingInst, int[] component)> ExtractBlockingSets()
        {
            foreach((int movingInst, var bitSetList) in blockingSets)
            {
                foreach(int[] component in bitSetList.ExtractBlockers(idxToBit))
                {
                    yield return (movingInst, component);
                }
            }
        }

        /// <summary>
        /// Unions 2 connected components.
        /// Adds all known blocking combinations of other to this.
        /// </summary>
        /// <param name="other"></param>
        public void Union(ConnectedComponentBlockers other)
        {
            int curCount = idxToBit.Count;
            // add all mappings upfront to prevent resizing twice
            foreach (int key in other.idxToBit.Keys)
            {
                idxToBit.TryAdd(key, idxToBit.Count);
            }

            foreach ((int otherNodeIdx, IBitSetList otherSet) in other.blockingSets)
            {
                IEnumerable<int[]> otherBlockers = otherSet.ExtractBlockers(other.idxToBit);
                foreach(int[] otherBlocker in otherBlockers)
                {
                    AddBlockerAllMappingsPresent(otherNodeIdx, otherBlocker);
                }
            }
        }

        public bool IsKnownBlocking(int nodeIdx, IEnumerable<int> blockingCombination)
        {
            if (!blockingSets.TryGetValue(nodeIdx, out IBitSetList blockers))
                return false;
            return blockers.IsKnownBlocking(blockingCombination.Append(nodeIdx), idxToBit);
        }

        /// <summary>
        /// Detect first cycle (if any) in the blocked sets that makes separation impossible
        /// </summary>
        public List<int> DeadlockedCycle(ICollection<int> component = null)
        {
            if (BlockerSetsCount < 3) return [];
            component ??= idxToBit.Keys;

            List<int> blocked = component.Where(blockingSets.ContainsKey).ToList();
            if (blocked.Count < 3) { blocked.Clear(); return blocked; }

            bool removed;
            do
            {
                removed = false;
                for (int i = blocked.Count - 1; i >= 0; i--)
                {
                    int moving = blocked[i];
                    if (!IsKnownBlocking(moving, blocked))
                    {
                        blocked.RemoveAt(i);
                        if (blocked.Count < 3) { blocked.Clear(); return blocked; }
                        removed = true;
                    }
                }
            } while (removed);

            return blocked;
        }

        public bool AddBlocker(int nodeIdx, IEnumerable<int> blockingCombination)
        {
            blockingCombination = blockingCombination.Append(nodeIdx);
            foreach (int idx in blockingCombination)
            {
                _ = idxToBit.TryAdd(idx, idxToBit.Count);
            }
            return AddBlockerAllMappingsPresent(nodeIdx, blockingCombination);
        }

        private bool AddBlockerAllMappingsPresent(int nodeIdx, IEnumerable<int> blockingCombination)
        {
            if (blockingSets.TryGetValue(nodeIdx, out IBitSetList blockers))
            {
                if (idxToBit.Count > blockers.MaxCount)
                {
                    // overflow, copy to wider type
                    IBitSetList widenedCopy = blockers.CopyWiden(idxToBit.Count);
                    blockingSets[nodeIdx] = widenedCopy;
                    blockers = widenedCopy;
                }
                return blockers.AddBlocker(blockingCombination, idxToBit);
            }
            else
            {
                blockers = NewBitSetList();
                bool added = blockers.AddBlocker(blockingCombination, idxToBit);
                Debug.Assert(added);
                blockingSets.Add(nodeIdx, blockers);
                return true;
            }
        }

        private IBitSetList NewBitSetList()
        {
            return idxToBit.Count switch
            {
                <= 64 => new BitSetList<BinaryIntegerBitSet<ulong>>(),
                <= 128 => new BitSetList<BinaryIntegerBitSet<UInt128>>(),
                <= 256 => new BitSetList<VectorBitSet<BitSet256>>(),
                <= 512 => new BitSetList<VectorBitSet<BitSet512>>(),
                <= 1024 => new BitSetList<VectorBitSet<BitSet1024>>(),
                <= 2048 => new BitSetList<VectorBitSet<BitSet2048>>(),
                > 2048 => new BitSetList<BinaryIntegerBitSet<BigInteger>>(),
            };
        }
    }
}
