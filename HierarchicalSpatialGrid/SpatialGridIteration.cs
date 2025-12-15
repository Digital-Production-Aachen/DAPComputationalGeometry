using g3;
using System;
using System.Collections.Generic;

namespace g3
{
    public interface ISpatialGridIterator<T> where T : IBoundsProvider
    {
        int NumLevels { get; }
        bool HasNextLevel { get; }
        double CurrentCellSize { get; }
        AxisAlignedBox3d QueryVolume { get; set; }
        int VisitedCount { get; }
        int IteratedCount { get; }
        SpatialGridIteratorState State { get; }
        Vector2d XyInterval { get; set; }

        IReadOnlyCollection<GridPosition<T>> IterateNextLevel();
        IReadOnlyCollection<GridPosition<T>> IterateLevel(int levelNr);
        void Reset();
        void SetVisited(GridPosition<T> position);
    }

    public readonly struct GridPosition<T> : IEquatable<GridPosition<T>> where T : IBoundsProvider
    {
        public readonly T pos;
        internal readonly int levelIdx;
        internal readonly byte instIdx;

        internal GridPosition(T pos, int levelIdx, byte instIdx)
        {
            this.pos = pos;
            this.levelIdx = levelIdx;
            this.instIdx = instIdx;
        }

        public static GridPosition<T> Invalid => new GridPosition<T>(default, -1, 0);

        public override bool Equals(object obj)
        {
            return obj is GridPosition<T> position && Equals(position);
        }

        public bool Equals(GridPosition<T> other)
        {
            return levelIdx == other.levelIdx &&
                   instIdx == other.instIdx;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(levelIdx, instIdx);
        }

        public static implicit operator T(GridPosition<T> gp) => gp.pos;

        public static bool operator ==(GridPosition<T> left, GridPosition<T> right)
        {
            return left.Equals(right);
        }

        public static bool operator != (GridPosition<T> left, GridPosition<T> right)
        {
            return !(left == right);
        }
    }

    public sealed class SpatialGridIteratorState
    {
        public readonly BitVector256[] visitedState;

        public SpatialGridIteratorState(BitVector256[] visitedState)
        {
            this.visitedState = visitedState;
        }
    }
}
