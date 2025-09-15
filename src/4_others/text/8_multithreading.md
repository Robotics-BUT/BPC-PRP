# Multithreading

## Motivation

Why use multiple threads?
 - Modern CPUs have multiple cores; using multiple threads can improve performance by performing tasks in parallel.
 - Some tasks, like handling multiple network connections, benefit from concurrent operations to remain responsive.

Key Concepts
 - Concurrency vs. Parallelism:
    - Concurrency is the composition of independently executing processes or threads.
    - Parallelism is the simultaneous execution of (possibly related) computations, using multiple physical CPU cores.
 - Threads: A thread is the smallest sequence of programmed instructions that can be managed independently by a scheduler.

## Risks & Challenges of Multithreading

 - Data Races: Two or more threads access a shared variable without proper synchronization, and at least one thread writes to the variable.
 - Deadlocks: Two or more threads are blocked forever, each waiting for the other to release a resource.
 - Race Conditions: A program’s outcome depends on the sequence of events or timings of threads.
 - Complexity: Debugging and reasoning about concurrent programs is generally harder than single-threaded ones.

## Basic Thread Creation and Management

### The `<thread>` Header

Modern C++ (C++11 and above) provides a standard way to create and manage threads through the `<thread>` header.

```c++
// Example 1: Creating a Simple Thread
#include <iostream>
#include <thread>

void helloFunction() {
    std::cout << "Hello from thread!\n";
}

int main() {
    std::thread t(helloFunction); // Create a thread running helloFunction
    t.join();                     // Wait for the thread to finish
    std::cout << "Hello from main!\n";
    return 0;
}
```

Explanation:
 - `std::thread t(helloFunction);` creates a new thread that executes `helloFunction`.
 - `t.join();` ensures the main thread waits until `t` finishes.
 - If you omit `t.join()`, the program may exit before the thread finishes, or you must call `t.detach()` if you intend the thread to run independently.

### Lambda Functions with Threads

Instead of passing a function pointer, you can also pass a lambda:

```c++
// Example 2: Using a Lambda
#include <iostream>
#include <thread>

int main() {
    std::thread t([](){
        std::cout << "Hello from a lambda thread!\n";
    });

    t.join();
    std::cout << "Hello from main!\n";
    return 0;
}
```

### Passing Arguments to Threads

You can pass arguments to the thread function by specifying them after the callable:
```c++
// Example 3: Passing Arguments
#include <iostream>
#include <thread>

void printValue(int x) {
    std::cout << "Value: " << x << "\n";
}

int main() {
    int num = 42;
    std::thread t(printValue, num);
    t.join();
    return 0;
}
```

### Detaching Threads
 - `t.detach()` makes the thread run independently; the main thread does not wait for it.
 - Use with caution: A detached thread can lead to tricky bugs if you rely on shared data in it.

## Synchronization Mechanisms

### Mutex and Lock Guards

To avoid data races, you typically protect shared data with a mutex. Only one thread can lock a mutex at a time.

```c++
// Example 4: Using std::mutex and std::lock_guard
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

std::mutex m;
int sharedCounter = 0;

void increment(int iterations) {
    for(int i = 0; i < iterations; ++i) {
        // Lock the mutex before modifying shared resource
        std::lock_guard<std::mutex> lock(m);
        ++sharedCounter;
    }
}

int main() {
    std::vector<std::thread> threads;
    for(int i = 0; i < 5; ++i) {
        threads.emplace_back(increment, 10000);
    }

    for(auto& t : threads) {
        t.join();
    }

    std::cout << "Final value of sharedCounter: " << sharedCounter << "\n";
    return 0;
}

```

Important Points:
 - `std::lock_guard<std::mutex>` automatically locks the mutex upon creation and unlocks it when it goes out of scope.
 - This prevents forgetting to unlock, especially in the presence of exceptions or multiple return statements.

### Unique Lock

`std::unique_lock<std::mutex>` is more flexible than std::lock_guard, allowing you to lock/unlock explicitly.

### Condition Variables

 - Condition variables allow threads to wait (block) until they are notified that some condition is true.
 - They typically work with a mutex to ensure correct data access.

```c++
// Example 5: Producer-Consumer with Condition Variables
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

std::mutex mtx;
std::condition_variable cv;
std::queue<int> dataQueue;
bool finished = false;

void producer() {
    for(int i = 1; i <= 5; ++i) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            dataQueue.push(i);
            std::cout << "Produced: " << i << "\n";
        }
        cv.notify_one(); // Notify one waiting thread
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Signal that production is finished
    {
        std::lock_guard<std::mutex> lock(mtx);
        finished = true;
    }
    cv.notify_all();
}

void consumer() {
    while(true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return !dataQueue.empty() || finished; });
        if(!dataQueue.empty()) {
            int value = dataQueue.front();
            dataQueue.pop();
            std::cout << "Consumed: " << value << "\n";
        }
        else if(finished) {
            break; // No more data
        }
    }
}

int main() {
    std::thread prod(producer);
    std::thread cons(consumer);

    prod.join();
    cons.join();

    return 0;
}

```

Explanation:
 - The producer thread pushes data to `dataQueue` and notifies the consumer.
 - The consumer thread waits (`cv.wait`) until it is notified that either new data is available or production is finished.
 - `cv.wait(lock, condition)` atomically unlocks the mutex and sleeps until `condition` is true, then locks the mutex again before returning.

### Atomic Operations

For simple operations like incrementing a counter, you can use `std::atomic` instead of a mutex:

```c++
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>

std::atomic<int> sharedCounter(0);

void increment(int iterations) {
    for(int i = 0; i < iterations; ++i) {
        ++sharedCounter;
    }
}

int main() {
    const int threadCount = 5;
    std::vector<std::thread> threads;

    for(int i = 0; i < threadCount; ++i) {
        threads.emplace_back(increment, 10000);
    }

    for(auto& t : threads) {
        t.join();
    }

    std::cout << "Final Counter: " << sharedCounter.load() << "\n";
    return 0;
}

```

Note: Atomic operations are typically more efficient than locking but only suitable for simple scenarios (increment, bitwise operations, etc.).

## Practical Examples and Exercise Ideas

### Summation of Large Array in Parallel

One common pattern is to split a task into chunks that multiple threads work on.

```c++
#include <iostream>
#include <thread>
#include <vector>
#include <numeric>

void partialSum(const std::vector<int>& data, int start, int end, long long& result) {
    long long sum = 0;
    for(int i = start; i < end; ++i) {
        sum += data[i];
    }
    result = sum;
}

int main() {
    // Example data
    std::vector<int> data(1000000, 1); // 1 million elements of value 1

    long long result1 = 0, result2 = 0;
    int mid = data.size() / 2;

    // Create 2 threads to handle half the data each
    std::thread t1(partialSum, std::cref(data), 0, mid, std::ref(result1));
    std::thread t2(partialSum, std::cref(data), mid, data.size(), std::ref(result2));

    t1.join();
    t2.join();

    long long total = result1 + result2;
    std::cout << "Total sum: " << total << "\n";
    return 0;
}
```

### Exercise: Extend the Summation
 - Modify the code to use four threads instead of two.
 - Compare performance for different numbers of threads and array sizes.
 - Explore usage of std::mutex or `std::atomic<long long>` if you want to accumulate into a single variable, but be mindful of performance overheads.

### Exercise: Calculate Pi Using Multiple Threads
 - Create multiple threads to estimate π by generating random points in a square and checking how many fall within the unit circle (Monte Carlo method).
 - Each thread returns the count of points inside the circle; combine results in the main thread and compute the approximation of π. 
 - Compare performance with different thread counts.

### Tips and Best Practices

 - Limit shared data
   - Minimize the portion of code that needs synchronization to reduce contention.

 - Avoid excessive locking
   - Use finer-grained locks or lock-free structures where applicable, but only if you fully understand the concurrency implications.

 - Use high-level concurrency abstractions if possible
   - For example, C++17’s std::async and std::future or higher-level frameworks can simplify concurrency.

 - Always check for data races
   - Tools like ThreadSanitizer can help detect concurrency issues.

 - Understand memory model
   - C++ has a well-defined memory model for atomic operations and synchronization.


## Resources
- C++ reference for threads: https://en.cppreference.com/w/cpp/thread
- C++ reference for mutexes and locks: https://en.cppreference.com/w/cpp/thread/mutex
- Condition variables: https://en.cppreference.com/w/cpp/thread/condition_variable
- Atomics and memory order: https://en.cppreference.com/w/cpp/atomic
- ThreadSanitizer (data race detector): https://github.com/google/sanitizers/wiki/ThreadSanitizer
