using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace g3
{
    public enum CacheState
    {
        NOT_COMPUTED,
        COMPUTE_IN_PROGRESS,
        COMPUTED
    }

    /// <summary>
    /// Implements a generic, multi threading save work queue.
    /// The work queue is filled single threaded by enumerating the workProducer.
    /// Any number of threads can join the computation at any time and execute workFunction on work queue items.
    /// WorkFunction is responsible for filling the cache.
    /// When all work is done, cache state is set to computed and all joined threads are released.
    /// </summary>
    /// <typeparam name="TWorkInput"></typeparam>
    /// <typeparam name="TCollection"></typeparam>
    /// <typeparam name="TValue"></typeparam>
    public class WorkQueueCache<TWorkInput, TCollection> : IDisposable
    {
        private TCollection _cachedValues;
        private CacheState _cacheState;
        private int _workCounter;
        private object mainLock = new();
        private ManualResetEventSlim AllWorkDone = new ManualResetEventSlim(false);
        private int _workGoal;
        private BlockingCollection<TWorkInput> workQueue;
        private bool disposedValue;
        private readonly WorkProducer workProducer;
        private readonly WorkFunction workFunction;
        private readonly Action finalAction;
        private readonly Action<bool> workQueueEmptyAction;
        private ExceptionDispatchInfo faultedException = null;

        public CacheState cacheState { get => _cacheState; }
        public bool IsAddingCompleted => workQueue.IsAddingCompleted;
        public int WorkGoal => _workGoal;
        public TCollection CachedValues { get => _cachedValues; }
        public TimeSpan ComputationTimeout = TimeSpan.FromMinutes(10);

        /// <summary>
        /// WorkFunction is executed on work queue items.
        /// WorkFunction is responsible for filling cacheForResult.
        /// </summary>
        /// <param name="cacheForResult">the cache to fill with results</param>
        /// <param name="workToDo">the work item to process</param>
        public delegate void WorkFunction(TCollection cacheForResult, TWorkInput workToDo);

        /// <summary>
        /// WorkProducer is executed to generate all work items that should be processes by WorkFunc.
        /// </summary>
        /// <returns></returns>
        public delegate IEnumerable<TWorkInput> WorkProducer();

        public WorkQueueCache(TCollection cache, WorkProducer workProducer, WorkFunction workFunction, Action finalActionSingleThread = null, Action<bool> workQueueEmptyAction = null)
        {
            this._cachedValues = cache;
            this.workProducer = workProducer ?? throw new ArgumentNullException(nameof(workProducer));
            this.workFunction = workFunction ?? throw new ArgumentNullException(nameof(workFunction));
            this.finalAction = finalActionSingleThread;
            this.workQueueEmptyAction = workQueueEmptyAction;
        }

        /// <summary>
        /// Ensures the cache is computed. If it is not (yet), the thread joins any ongoing computation until it is computed.
        /// If blocking is set to false, the call returns when there is no more work to parallelize, but the computation might be ongoing on other threads.
        /// </summary>
        public void EnsureComputedOrJoinComputation(bool blocking = true)
        {
            try
            {
                if (_cacheState != CacheState.COMPUTED)
                {
                    bool IAmMain = false;
                    if (_cacheState == CacheState.NOT_COMPUTED)
                    {
                        //first thread who wins the lock is promoted to main thread
                        lock (mainLock)
                        {
                            IAmMain = _cacheState == CacheState.NOT_COMPUTED;
                            if (IAmMain)
                            {
                                AllWorkDone.Reset();
                                workQueue = new BlockingCollection<TWorkInput>(new ConcurrentQueue<TWorkInput>());
                                _cacheState = CacheState.COMPUTE_IN_PROGRESS;
                                _workCounter = 0;
                                _workGoal = 0;
                            }
                            //quickly release the lock to free other waiting threads
                        }
                    }

                    if (IAmMain)
                    {
                        //Main thread code: Producer
                        foreach (var work in workProducer())
                        {
                            workQueue.Add(work);
                            _workGoal++;
                            faultedException?.Throw();
                        }
                        workQueue.CompleteAdding();
                    }

                    while (!workQueue.IsCompleted)
                    {
                        //worker thread code: Consumer
                        faultedException?.Throw();
                        if (workQueue.TryTake(out var work, 100))
                        {
                            faultedException?.Throw();
                            workFunction(_cachedValues, work);
                            Interlocked.Increment(ref _workCounter);
                        }
                    }

                    faultedException?.Throw();
                    workQueueEmptyAction?.Invoke(blocking);
                    faultedException?.Throw();

                    //wait until all threads are done with work they took
                    if (_workGoal > _workCounter)
                    {
                        if (blocking) 
                        { 
                            bool belowTimeout = AllWorkDone.Wait(ComputationTimeout);
                            if (!belowTimeout) { throw new TimeoutException("Timeout waiting for other threads to complete work"); }
                        }
                        faultedException?.Throw();
                    }
                    else
                    {
                        faultedException?.Throw();
                        //this thread did the last computation
                        if (finalAction != null)
                        {
                            TimeSpan timeout = blocking ? ComputationTimeout : TimeSpan.Zero;
                            //lock to ensure final action is done exactly once
                            //and threads are released only after final action is done (if blocking)
                            if (Monitor.TryEnter(finalAction, timeout))
                            {
                                try
                                {
                                    faultedException?.Throw();
                                    if (_cacheState != CacheState.COMPUTED)
                                    {
                                        finalAction();
                                        _cacheState = CacheState.COMPUTED;
                                        AllWorkDone.Set();
                                    }
                                }
                                finally
                                {
                                    Monitor.Exit(finalAction);
                                }
                            }
                            else if (blocking) { throw new TimeoutException("Timeout waiting for other thread to complete final action"); }
                        }
                        else
                        {
                            //no problem in doing this more than once, no lock necessary, free all waiting threads
                            _cacheState = CacheState.COMPUTED;
                            AllWorkDone.Set();
                        }
                    }
                }
            }
            catch (Exception e)
            {
                //capture any unhandled exception info to additionally dispatch it from all other threads waiting for completion
                //ExceptionDispatchInfo preserves the original stack trace when re-thrown from other threads
                faultedException = ExceptionDispatchInfo.Capture(e);
                AllWorkDone.Set();
                throw;
            }
        }

        private Task JoinAsync() => Task.Run(() => EnsureComputedOrJoinComputation(false));
        public Task PreComputeAsyncParallel(int maxDegreeOfParallelism = -1)
        {
            if (_cacheState == CacheState.COMPUTED || maxDegreeOfParallelism == 0)
                return Task.CompletedTask;

            int degreeOfParallelism = maxDegreeOfParallelism == -1 ?
                Environment.ProcessorCount - 1
                : Math.Clamp(1, maxDegreeOfParallelism, Environment.ProcessorCount);

            if (degreeOfParallelism == 1)
                return JoinAsync();

            var tasks = new Task[degreeOfParallelism];
            for (int i = 0; i < degreeOfParallelism; i++)
            {
                tasks[i] = JoinAsync();
            }
            return Task.WhenAll(tasks);
        }

        public void Clear()
        {
            _cacheState = CacheState.NOT_COMPUTED;
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    AllWorkDone?.Dispose();
                    workQueue?.Dispose();
                }

                _cachedValues = default;
                disposedValue = true;
            }
        }

        public void Dispose()
        {
            // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
