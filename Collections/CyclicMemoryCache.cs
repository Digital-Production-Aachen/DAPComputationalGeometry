using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;

namespace g3
{
    /// <summary>
    /// A cyclic memory cache based on dictionary.
    /// Tracks the size of entries added using the provided sizeFunc. Retains two cache segments, primary and second chance.
    /// When size or entry count limits are hit, the primary cache segment is demoted into the readonly second chance (segment rotation).
    /// Entries are copied from second chance to primary upon access. All entries still in second chance are discarded when segment rotation happens.
    /// </summary>
    /// <typeparam name="TKey"></typeparam>
    /// <typeparam name="TValue"></typeparam>
    public class CyclicMemoryCache<TKey, TValue>
    {
        private class CacheSegment
        {
            private Dictionary<TKey, TValue> _dict;
            private long _sizeInBytes = 0;
            private readonly Func<TValue, int> _sizeFunc;

            public CacheSegment(Func<TValue, int> sizeFunc, int capacity)
            {
                _sizeFunc = sizeFunc;
                _dict = new(Convert.ToInt32(1.5 * capacity));
            }

            public long SizeInBytes => _sizeInBytes;
            public double AverageEntrySize => SizeInBytes / Count;
            public int Count => _dict.Count;

            public bool TryGetValue(TKey key, out TValue value) => _dict.TryGetValue(key, out value);

            public bool TryAdd(TKey key, TValue value)
            {
                if (_dict.TryAdd(key, value))
                {
                    _sizeInBytes += _sizeFunc(value);
                    return true;
                }
                return false;
            }

            public bool TryRemove(TKey key)
            {
                if (_dict.Remove(key, out var removed))
                {
                    _sizeInBytes -= _sizeFunc(removed);
                    return true;
                }
                return false;
            }

            public void Clear()
            {
                _dict.Clear();
                _sizeInBytes = 0;
            }
        }

        //implements a 3 segment rotations clearing
        //primary segment, reads and adds go here
        private CacheSegment _primary;
        //second chance segment, only reads go here, entries are promoted to primary and removed
        private CacheSegment _secondChance;
        private long _maxBytes;

        // Size accessor delegate
        private readonly Func<TValue, int> _sizeFunc;
        private int initCapacity;

        public CyclicMemoryCache(long maxBytes, Func<TValue, int> calculateByteSize, int capacity = 4)
        {
            _sizeFunc = calculateByteSize ?? throw new ArgumentNullException(nameof(calculateByteSize));
            this.initCapacity = capacity;
            _primary = new CacheSegment(calculateByteSize, initCapacity);
            _maxBytes = maxBytes;
        }

        public int Count => _primary.Count + _secondChance?.Count ?? 0;

        public long SizeInBytes => _primary.SizeInBytes + _secondChance?.SizeInBytes ?? 0;

        public long MaxSizeInBytes
        {
            get => _maxBytes;
            set
            {
                ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, nameof(MaxSizeInBytes));
                _maxBytes = value;
            }
        }

        public int MaxCapacity { get => initCapacity; set => initCapacity = value; }

        public bool TryGetValue(TKey key, out TValue value)
        {
            if (_primary.TryGetValue(key, out value)) return true;
            if (_secondChance is null) return false;//fast path for small working sets
            if (_secondChance.TryGetValue(key, out value))
            {
                //promote value to primary again on access
                //because we remove the value from second chance, we cannot exceed the size limit here
                //and don't have to evict
                //this lets the primary grow larger than half and use more from _maxBytes if all requested values are in secondary
                //meaning the working set is below _maxBytes
                if (_primary.TryAdd(key, value))
                {
                    _secondChance.TryRemove(key);
                }
            }
            return false;
        }

        public bool TryAdd(TKey key, TValue value)
        {
            if (_primary.TryAdd(key, value))
            {
                EvictIfLimitHit(value);
                return true;
            }
            return false;
        }

        public void Clear()
        {
            _primary.Clear();
            _secondChance.Clear();
        }

        private bool EvictIfLimitHit(TValue value)
        {
            long primarySize = _primary.SizeInBytes;
            int added = _primary.Count;
            if (primarySize <= _maxBytes / 2 && added < initCapacity)
                return false;

            CacheSegment oldPrimary = _primary;
            _secondChance?.Clear();
            _primary = _secondChance ?? new CacheSegment(_sizeFunc, initCapacity);
            _secondChance = oldPrimary;

            return true;
        }
    }
}
