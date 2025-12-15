using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace g3
{
    /// <summary>
    /// Represents a 3D hierarchical spatial grid of Ts.
    /// Uses axis aligned bounding boxes for proximity detection.
    /// Ts are organized in a hierarchy of variable resolution grids, according to their AABB size.
    /// Grid cell coverage is stored implicitly using dense bit field arrays for x,y,z rows and columns.
    /// The data structure can be iterated thread safe for multiple concurrent readers, but only one single writer.
    /// Iterators reflect most concurrent writes to the grid while iterating. But this is not guarantied, each grid level has 
    /// a capacity of 256 instances. If the capacity of any level is exceeded, a copy on write is performed and further writes to the same level
    /// do not reflect in previously acquired iterators.
    /// </summary>
    public class HierarchicalSpatialGrid<T> : IReadOnlyCollection<T> where T:IBoundsProvider
    {
        // the levels list is read an append only, to keep any GridPositions handed out by iterators always valid for setting visited states
        // and keeping any cached visited states valid. for this the level index is indirected once by the level object (levelIdx field).
        // levels uses copy on write for level insert operations, changing the order of levels.
        // Iterators use a snapshot of levels that does not reflect level inserts during iteration. The snapshot gets updated on iterator creation and reset.
        volatile ReadAndAddOnlyList<SpatialGridLevel> levels = new();
        AxisAlignedBox3d gridBounds;
        public int CurrentLevelDepth { get; private set; }
        public int MaxLevelDepth { get; set; } = 7;
        public int FullLevels => levels.Count - CurrentLevelDepth;
        public int Count => levels.Sum(x => x.Count);

        public HierarchicalSpatialGrid(AxisAlignedBox3d bounds)
        {
            this.gridBounds = bounds;
            levels.Add(new SpatialGridLevel(gridBounds, gridBounds.MinDim() / 2, 0));
            CurrentLevelDepth = 1;
        }

        public void AppendInstance(T position)
        {
            var posBounds = position.Bounds;
            var minDim = posBounds.MinDimXY();
            for (int i = 0; i < levels.Count; i++)
            {
                SpatialGridLevel level = levels[i];
                if (level.CellSize < minDim)
                {
                    int levelIdx = i - 1;
                    if (levelIdx < 0)
                    {
                        levelIdx = 0;
                        double firstSize = levels[0].CellSize;
                        while (levelIdx < levels.Count - 1 && levels[levelIdx + 1].CellSize == firstSize) { levelIdx++; }
                    }
                    level = levels[levelIdx];
                    if (level.Count < SpatialGridLevel.MaxCount)
                    {
                        level.AppendInstance(position);
                        return;
                    }
                    else
                    {
                        //level is full, insert new same size level
                        var newLevel = InsertNewLevel(level.CellSize, levelIdx + 1);
                        newLevel.AppendInstance(position);
                        return;
                    }
                }
            }

            //no suitable level added yet
            SpatialGridLevel lastLevel;
            for (int i = levels.Count; CurrentLevelDepth < MaxLevelDepth; i++)
            {
                // caution, updating an iterator from a historical state depends on this insert and append behavior implementation
                lastLevel = levels[^1];
                var newLevel = AppendNewLevel(lastLevel.CellSize / 2);
                CurrentLevelDepth++;
                if (newLevel.CellSize < minDim)
                {
                    newLevel.AppendInstance(position);
                    return;
                }
            }

            //reached max depth
            lastLevel = levels[^1];
            if (lastLevel.Count < SpatialGridLevel.MaxCount)
            {
                lastLevel.AppendInstance(position);
            }
            else
            {
                var newLevel = AppendNewLevel(lastLevel.CellSize);
                newLevel.AppendInstance(position);
            }
        }

        private SpatialGridLevel AppendNewLevel(double cellSize)
        {
            var newLevel = new SpatialGridLevel(gridBounds, cellSize, levels.Count);
            levels.Add(newLevel);
            //we do not need copy on write for append
            //if we have never inserted a level, levelIterator and levels are the same object
            if (!ReferenceEquals(levels, levels)) { levels.Add(newLevel); }
            return newLevel;
        }

        private SpatialGridLevel InsertNewLevel(double cellSize, int insertIdx)
        {
            var newLevel = new SpatialGridLevel(gridBounds, cellSize, levels.Count);
            //copy on write for insert
            ReadAndAddOnlyList<SpatialGridLevel> newLevels = new(levels.Capacity);
            for (int i = 0; i < insertIdx; i++)
            {
                newLevels.Add(levels[i]);
            }
            newLevels.Add(newLevel);
            for (int i = insertIdx; i < levels.Count; i++)
            {
                newLevels.Add(levels[i]);
            }
            levels.Add(newLevel);
            levels = newLevels;
            return newLevel;
        }

        public IEnumerable<T> CollisionCandidates(AxisAlignedBox3d queryVolume)
        {
            var iter = GetIterator(queryVolume);
            while (iter.HasNextLevel)
            {
                foreach (var pos in iter.IterateNextLevel())
                {
                    yield return pos.pos;
                }
            }
        }

        public ISpatialGridIterator<T> GetIterator(in AxisAlignedBox3d queryVolume, SpatialGridIteratorState stateSnapshot = null)
        {
            return new SpatialGridIterator(this, queryVolume, stateSnapshot?.visitedState);
        }

        public ISpatialGridIterator<T> GetXYCheckerboardIterator(in AxisAlignedBox3d queryVolume, Vector2d xyInterval, SpatialGridIteratorState stateSnapshot = null)
        {
            return new SpatialGridXYSelectIterator(this, queryVolume, xyInterval, stateSnapshot?.visitedState);
        }

        public IEnumerator<T> GetEnumerator()
        {
            return levels.SelectMany(x => x.Instances).GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        private class SpatialGridIterator : ISpatialGridIterator<T>
        {
            protected readonly HierarchicalSpatialGrid<T> hGrid;
            // the current levels iteration order snapshot
            protected IReadOnlyList<SpatialGridLevel> levels;
            protected readonly List<BitVector256> visitedState;
            protected int currentLevelIdx = -1;
            protected AxisAlignedBox3d _queryVolume;
            protected Vector2d _xyInterval;

            public int NumLevels => levels.Count;
            public virtual bool HasNextLevel => currentLevelIdx < levels.Count - 1;
            public double CurrentCellSize => levels[currentLevelIdx].CellSize;
            public int VisitedCount => visitedState.Sum(x => x.PopulationCount);
            public int IteratedCount => _curIterationCount;
            public int Count => levels.Sum(x => x.Count);
            protected int _curIterationCount = 0;
            public SpatialGridIteratorState State
            {
                get
                {
                    int snapShotCount = levels.Count;
                    while (visitedState.Count < snapShotCount) { visitedState.Add(BitVector256.Zero); }
                    return new SpatialGridIteratorState(visitedState.ToArray());
                }
            }

            internal SpatialGridIterator(HierarchicalSpatialGrid<T> hGrid, AxisAlignedBox3d queryVolume, BitVector256[] visitedState = null)
            {
                this.hGrid = hGrid;
                this.visitedState = new List<BitVector256>(hGrid.levels.Count + 1);
                if (visitedState != null)
                {
                    this.visitedState.AddRange(visitedState);
                }
                _queryVolume = queryVolume;
                Reset();
            }

            public AxisAlignedBox3d QueryVolume
            {
                get => _queryVolume;
                set
                {
                    Reset();
                    _queryVolume = value;
                }
            }

            public Vector2d XyInterval { get => _xyInterval; set => _xyInterval = value; }

            public virtual IReadOnlyCollection<GridPosition<T>> IterateNextLevel()
            {
                currentLevelIdx++;
                var level = levels[currentLevelIdx];//index range check included
                while (visitedState.Count < levels.Count) { visitedState.Add(BitVector256.Zero); }
                var visitedSpan = CollectionsMarshal.AsSpan(visitedState);
                var levelIter = level.CollisionCandidates(ref _queryVolume, ref visitedSpan[level.levelIdx], out int countSnapShot, out var visitVec, Vector3i.One, Vector3i.Zero);
                _curIterationCount += countSnapShot;
                return levelIter;
            }

            public IReadOnlyCollection<GridPosition<T>> IterateLevel(int levelNr)
            {
                currentLevelIdx = levelNr - 1;
                return IterateNextLevel();
            }

            public virtual void Reset()
            {
                currentLevelIdx = -1;
                _curIterationCount = 0;
                // update the levels iterator to the latest version published by the grid
                levels = hGrid.levels;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetVisited(GridPosition<T> position)
            {
                var visitedSpan = CollectionsMarshal.AsSpan(visitedState);
                if (visitedSpan[position.levelIdx][position.instIdx] == true) throw new ArgumentException("GridPosition already set visited");
                visitedSpan[position.levelIdx][position.instIdx] = true;
            }
        }

        private class SpatialGridXYSelectIterator : SpatialGridIterator
        {
            private BitVector256 iteratedMask;
            private int quadrant;

            internal SpatialGridXYSelectIterator(HierarchicalSpatialGrid<T> hGrid, AxisAlignedBox3d queryVolume, Vector2d xyInterval, BitVector256[] visitedState = null) : base(hGrid, queryVolume, visitedState)
            {
                this._xyInterval = xyInterval;
            }

            public override bool HasNextLevel => quadrant > 0 || currentLevelIdx < levels.Count - 1;

            public override IReadOnlyCollection<GridPosition<T>> IterateNextLevel()
            {                
                if (quadrant == 0)
                {
                    currentLevelIdx++;
                    if (levels[currentLevelIdx].Count == 0) return Array.Empty<GridPosition<T>>();
                    if (_xyInterval.Abs().MinCoord() < CurrentCellSize * 2) 
                    {
                        currentLevelIdx--;
                        return base.IterateNextLevel();
                    }
                }

                // iterate the grid in intervals, selecting only a subset of the objects (overlapping a sparse cell grid)
                // repeat up to 3 times, selecting in quadrants
                // if we should further select after 4 sparse quadrant selections, the grid seems very sparse => return all remaining objects
                // this limits the selection overhead to constant time
                Vector3i intervals = new Vector3i();
                Vector3i offsets = new Vector3i();
                intervals.x = quadrant == 4 ? 1 : Math.Max(1, Convert.ToInt32(Math.Ceiling(Math.Abs(_xyInterval.x) / CurrentCellSize)));
                intervals.y = quadrant == 4 ? 1 : Math.Max(1, Convert.ToInt32(Math.Ceiling(Math.Abs(_xyInterval.y) / CurrentCellSize)));
                intervals.z = 1;
                offsets.x = quadrant switch
                {
                    0 => 0,
                    1 => intervals.x / 2,
                    2 => intervals.x / 2,
                    3 => 0,
                    4 => 0,
                    _ => throw new NotImplementedException(),
                };
                offsets.y = quadrant switch
                {
                    0 => 0,
                    1 => intervals.y / 2,
                    2 => 0,
                    3 => intervals.y / 2,
                    4 => 0,
                    _ => throw new NotImplementedException(),
                };
                offsets.z = 0;

                SpatialGridLevel curLevel = levels[currentLevelIdx];
                while (visitedState.Count < levels.Count) { visitedState.Add(BitVector256.Zero); }
                BitVector256 visited = visitedState[curLevel.levelIdx];
                visited |= iteratedMask;
                var signedIntervals = intervals * new Vector3i(Math.Sign(_xyInterval.x), Math.Sign(_xyInterval.y), 1);
                var levelIter = curLevel.CollisionCandidates(ref _queryVolume, ref visited, out int countSnapShot, out BitVector256 iterationMask, signedIntervals, offsets);
                _curIterationCount += iterationMask.PopulationCount;
                iteratedMask |= iterationMask;

                if (levels[currentLevelIdx].Count == iteratedMask.PopulationCount) 
                {
                    quadrant = 0;
                    iteratedMask = BitVector256.Zero;
                    return levelIter;
                }

                if (quadrant == 4) iteratedMask = BitVector256.Zero;
                quadrant = (quadrant, intervals.x > 1, intervals.y > 1) switch
                {
                    (_, false, false) => 0,
                    (0, true, _) => 1,
                    (0, _, true) => 1,
                    (1, true, _) => 2,
                    (1, false, _) => 3,
                    (2, _, true) => 3,
                    (2, _, false) => 4,
                    (3, _, _) => 4,
                    (4, _, _) => 0,
                    ( > 3 or < 0, _, _) => throw new InvalidOperationException("invalid state")
                };
                return levelIter;
            }

            public override void Reset()
            {
                base.Reset();
                quadrant = 0;
                iteratedMask = BitVector256.Zero;
            }
        }

        /// <summary>
        /// One level of the spatial grid hierarchy.
        /// </summary>
        private class SpatialGridLevel
        {
            ReadAndAddOnlyList<T> _instances = new();
            BitVector256[] x_bitfields;
            BitVector256[] y_bitfields;
            BitVector256[] z_bitfields;
            private Vector3d origin;
            private readonly double _cellSize;
            internal readonly int levelIdx;

            internal const int MaxCount = 256;

            internal int Count => _instances.Count;
            internal double CellSize => _cellSize;
            internal IReadOnlyList<T> Instances => _instances;


            internal SpatialGridLevel(AxisAlignedBox3d bounds, double cellSize, int levelIdx)
            {
                if (!(cellSize > 0)) throw new ArgumentException($"cell size must be positive, but is {cellSize}");
                this.levelIdx = levelIdx;
                this._cellSize = cellSize;
                this.origin = bounds.Min;
                Vector3d extent = bounds.Max - origin;
                x_bitfields = new BitVector256[Convert.ToInt32(Math.Ceiling(extent.x / cellSize))];
                y_bitfields = new BitVector256[Convert.ToInt32(Math.Ceiling(extent.y / cellSize))];
                z_bitfields = new BitVector256[Convert.ToInt32(Math.Ceiling(extent.z / cellSize))];
            }

            internal void AppendInstance(T position)
            {
                Debug.Assert(_instances.Count <= MaxCount);
                var bounds = position.Bounds;
                int idx = _instances.Count;
                SetBitfieldRangeTrueAt(idx, ref bounds, 0);
                SetBitfieldRangeTrueAt(idx, ref bounds, 1);
                SetBitfieldRangeTrueAt(idx, ref bounds, 2);
                _instances.Add(position);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private void SetBitfieldRangeTrueAt(int idx, ref AxisAlignedBox3d bounds, int dim)
            {
                var bitfield = MinMaxIdx(ref bounds, dim, out int i_min, out int i_max);
                for (int i = i_min; i <= i_max; i++)
                {
                    bitfield[i][idx] = true;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            internal BitVector256[] MinMaxIdx(ref AxisAlignedBox3d bounds, int dim, out int i_min, out int i_max)
            {
                var min = bounds.Min[dim] - origin[dim];
                var max = bounds.Max[dim] - origin[dim];
                i_min = Convert.ToInt32(Math.Floor(min / _cellSize));
                i_max = Convert.ToInt32(Math.Ceiling(max / _cellSize));
                var bitfield = GetBitVector(dim);
                i_min = int.Clamp(i_min, 0, bitfield.Length - 1);
                i_max = int.Clamp(i_max, 1, bitfield.Length - 1);
                return bitfield;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private BitVector256[] GetBitVector(int dim) => dim switch { 0 => x_bitfields, 1 => y_bitfields, _ => z_bitfields, };

            internal IReadOnlyCollection<GridPosition<T>> CollisionCandidates(ref AxisAlignedBox3d queryVolume, ref BitVector256 visitedMask, out int countSnapShot, out BitVector256 iterationMask, Vector3i intervals, Vector3i offsets)
            {
                countSnapShot = Count;
                if (countSnapShot == 0 || visitedMask.PopulationCount == countSnapShot) { iterationMask = BitVector256.Zero; return Array.Empty<GridPosition<T>>(); }
                //make sure we do not read beyond count, state might be inconsistent there 
                iterationMask = ~(visitedMask | BitVector256.OnesFrom(countSnapShot));
                const int z = 2;
                iterationMask = SpatialBitIntersection(z, ref queryVolume, iterationMask, intervals.z, offsets.z);
                if (!iterationMask.Any()) { return Array.Empty<GridPosition<T>>(); }
                return XYCollisionCandidates(ref queryVolume, ref iterationMask, new Vector2i(intervals.x, intervals.y), new Vector2i(offsets.x, offsets.y));
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private BitVector256 SpatialBitIntersection(int dim, ref AxisAlignedBox3d queryVolume, BitVector256 notVisited, int interval, int offset)
            {
                var bitfield = MinMaxIdx(ref queryVolume, dim, out int i_min, out int i_max);
                //perf early out: bounds cover all cells? The loop is 2^levelDepth iterations
                if (i_min == 0 && i_max == bitfield.Length - 1 && offset == 0 && Math.Abs(interval) == 1) { return notVisited; }

                //bit field intersection: objects present in coordinate cell interval and not visited
                if(interval > 0) //forwards loop
                {
                    i_min += offset;
                    i_min = Math.Min(i_min, i_max);
                    BitVector256 posVec = bitfield[i_min];
                    i_min += interval;
                    for (int i = i_min; i <= i_max; i += interval)
                    {
                        posVec |= bitfield[i];
                    }
                    notVisited &= posVec;
                }
                else             //backwards loop
                {
                    i_max -= offset;
                    i_max = Math.Max(i_min, i_max);
                    BitVector256 posVec = bitfield[i_max];
                    i_max += interval;
                    for (int i = i_max; i >= i_min; i += interval)
                    {
                        posVec |= bitfield[i];
                    }
                    notVisited &= posVec;
                }
                
                return notVisited;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private IReadOnlyCollection<GridPosition<T>> XYCollisionCandidates(ref AxisAlignedBox3d queryVolume, ref BitVector256 notVisited, Vector2i intervals, Vector2i offsets)
            {
                const int x = 0, y = 1;
                notVisited = SpatialBitIntersection(x, ref queryVolume, notVisited, intervals.x, offsets.x);
                if (!notVisited.Any()) { return Array.Empty<GridPosition<T>>(); }
                notVisited = SpatialBitIntersection(y, ref queryVolume, notVisited, intervals.y, offsets.y);
                if (!notVisited.Any()) { return Array.Empty<GridPosition<T>>(); }
                return new GridPositionCollection(this, notVisited);
            }

            private struct GridPositionCollection : IReadOnlyCollection<GridPosition<T>>, IEnumerator<GridPosition<T>>
            {
                SpatialGridLevel sourceLevel;
                BitVector256 bitField;
                ulong _currentBits;
                GridPosition<T> _current;
                int _nextIdx;

                public GridPositionCollection(SpatialGridLevel sourceLevel, BitVector256 bitField) : this()
                {
                    this.sourceLevel = sourceLevel;
                    this.bitField = bitField;
                }

                public int Count => bitField.PopulationCount;

                public GridPosition<T> Current => _current;

                object IEnumerator.Current => _current;

                public void Dispose() { }

                public IEnumerator<GridPosition<T>> GetEnumerator() => this;

                IEnumerator IEnumerable.GetEnumerator() => this;

                public bool MoveNext()
                {
                    while (_currentBits == 0)
                    {
                        if (_nextIdx >= 4) { return false; }
                        _currentBits = bitField.GetULong(_nextIdx++);
                    }
                    int idx = BitOperations.TrailingZeroCount(_currentBits) + (_nextIdx - 1) * 64;
                    _current = new GridPosition<T>(sourceLevel._instances[idx], sourceLevel.levelIdx, (byte)idx);
                    _currentBits &= _currentBits - 1;
                    return true;
                }

                public void Reset()
                {
                    _currentBits = bitField.GetULong(0);
                    _nextIdx = 1;
                }
            }
        }
    }
}
