using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;

namespace g3
{
    /// <summary>
    /// A concurrent cyclic memory cache based on ConcurrentDictionary.
    /// Tracks the size of entries added using the provided sizeFunc. Retains two cache segments, primary and second chance.
    /// When size or entry count limits are hit, the primary cache segment is demoted into the readonly second chance (segment rotation).
    /// Entries are copied from second chance to primary upon access. All entries still in second chance are discarded when segment rotation happens.
    /// </summary>
    /// <typeparam name="TKey"></typeparam>
    /// <typeparam name="TValue"></typeparam>
    public class ConcurrentCyclicMemoryCache<TKey, TValue>
    {
        private class CacheSegment
        {
            private ConcurrentDictionary<TKey, TValue> _dict;
            private long _sizeInBytes = 0;
            private int _addedCount = 0;
            private readonly Func<TValue, int> _sizeFunc;

            public CacheSegment(Func<TValue, int> sizeFunc, int concurrencyLevel, int capacity)
            {
                _sizeFunc = sizeFunc;
                _dict = new(concurrencyLevel, Convert.ToInt32(1.5 * capacity));
            }

            public long SizeInBytes => Volatile.Read(ref _sizeInBytes);
            public int AddedCount => Volatile.Read(ref _addedCount);
            public double AverageEntrySize => SizeInBytes / AddedCount;
            public int Count => _dict.Count;

            public bool TryGetValue(TKey key, out TValue value) => _dict.TryGetValue(key, out value);

            public bool TryAdd(TKey key, TValue value)
            {
                if (_dict.TryAdd(key, value))
                {
                    Interlocked.Add(ref _sizeInBytes, _sizeFunc(value));
                    Interlocked.Increment(ref _addedCount);
                    return true;
                }
                return false;
            }

            public bool TryRemove(TKey key)
            {
                if (_dict.TryRemove(key, out var removed))
                {
                    //do not track removed size, we currently don't need it
                    //Interlocked.Add(ref _sizeInBytes, -_sizeFunc(removed));
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
        private volatile CacheSegment _primary;
        //second chance segment, only reads go here, entries are promoted to primary and removed
        private volatile CacheSegment _secondChance;
        private long _maxBytes;

        private readonly object _evictLock = new object();
        // Size accessor delegate
        private readonly Func<TValue, int> _sizeFunc;
        private readonly int concurrencyLevel;
        private int initCapacity;

        public ConcurrentCyclicMemoryCache(long maxBytes, Func<TValue, int> calculateByteSize) 
            : this(maxBytes, calculateByteSize, Environment.ProcessorCount, 31) { }

        public ConcurrentCyclicMemoryCache(long maxBytes, Func<TValue, int> calculateByteSize, int concurrencyLevel, int capacity)
        {
            _sizeFunc = calculateByteSize ?? throw new ArgumentNullException(nameof(calculateByteSize));
            this.concurrencyLevel = concurrencyLevel;
            this.initCapacity = capacity;
            _primary = new CacheSegment(calculateByteSize, concurrencyLevel, initCapacity);
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
            lock (_evictLock)
            {
                //second chance first, it is readonly
                _secondChance = null;
                _primary = new CacheSegment(_sizeFunc, concurrencyLevel, initCapacity);
            }
        }

        private bool EvictIfLimitHit(TValue value)
        {
            long primarySize = _primary.SizeInBytes;
            int added = _primary.AddedCount;
            if (primarySize <= _maxBytes / 2 && added < initCapacity)
                return false;

            if (!Monitor.TryEnter(_evictLock))
                return false;

            try
            {
                CacheSegment oldPrimary = _primary;
                _primary = new CacheSegment(_sizeFunc, concurrencyLevel, initCapacity);
                _secondChance = oldPrimary;
                //old _secondChance will now be garbage collected, as soon as all readers are finished with it
            }
            finally
            {
                Monitor.Exit(_evictLock);
            }
            return true;
        }
    }
}
