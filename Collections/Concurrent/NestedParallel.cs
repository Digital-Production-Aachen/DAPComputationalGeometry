using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace g3
{
    public static class NestedParallel
    {
        private class NestedParallelState<T>
        {
            private readonly IReadOnlyList<T> items;
            Action<T> joinAction;
            private int workCounter = -1;

            public NestedParallelState(IReadOnlyList<T> items, Action<T> joinAction)
            {
                this.items = items;
                this.joinAction = joinAction;
            }

            public void WorkLoop()
            {
                //first stage: process all items single threaded
                int next;
                while ((next = Interlocked.Increment(ref workCounter)) < items.Count)
                {
                    joinAction(items[next]);
                }
                //second stage: we ran out of items, join the computation multi threaded per item
                foreach(var item in items)
                {
                    joinAction(item);
                }
            }
        }

        /// <summary>
        /// Executes join action over the provided list of items in 2 phases, limiting the degree of parallelism.
        /// 1. single threaded phase: joinAction is invoked once on each item
        /// 2. multi threaded join phase: when all item processing has been invoked once, available threads will join the compute on unfinished items
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="items">items to process with join action</param>
        /// <param name="joinAction">thread safe action to start or join a computation on an item (e.g. workQueueCache implementation)</param>
        /// <param name="maxDegreeOfParallelism">the maximum degree of parallelism to use</param>
        /// <returns>a task that completes when all parallel computations have completed</returns>
        public static System.Threading.Tasks.Task FlattenParallelCompute<T>(IReadOnlyList<T> items, Action<T> joinAction, int maxDegreeOfParallelism = -1)
        {
            if (items.Count == 0) { return Task.CompletedTask; }
            int degreeOfParallelism = maxDegreeOfParallelism == -1 ? Environment.ProcessorCount
                : Math.Clamp(1, maxDegreeOfParallelism, Environment.ProcessorCount);
            var state = new NestedParallelState<T>(items, joinAction);
            var tasks = new System.Threading.Tasks.Task[degreeOfParallelism];
            for (int i = 0; i < degreeOfParallelism; i++)
            {
                tasks[i] = Task.Run(state.WorkLoop);
            }
            return Task.WhenAll(tasks);
        }
    }
}
