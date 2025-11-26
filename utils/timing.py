"""
Timing utilities for control loops.

Provides rate limiting and loop timing.
"""

import time
import asyncio


class RateTimer:
    """Maintains a fixed control loop rate."""

    def __init__(self, hz: float):
        """
        Initialize rate timer.

        Args:
            hz: Desired loop frequency (Hz)
        """
        self.hz = hz
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        """Sleep to maintain loop rate."""
        current_time = time.time()
        elapsed = current_time - self.last_time

        if elapsed < self.period:
            time.sleep(self.period - elapsed)

        self.last_time = time.time()

    async def sleep_async(self):
        """Async sleep to maintain loop rate."""
        current_time = time.time()
        elapsed = current_time - self.last_time

        if elapsed < self.period:
            await asyncio.sleep(self.period - elapsed)

        self.last_time = time.time()

    def elapsed(self) -> float:
        """Get elapsed time since last call."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        return dt

    def reset(self):
        """Reset timer."""
        self.last_time = time.time()


class LoopTimer:
    """Measures loop execution time and statistics."""

    def __init__(self, name: str = "Loop"):
        """
        Initialize loop timer.

        Args:
            name: Name for this timer
        """
        self.name = name
        self.start_time = None
        self.durations = []
        self.max_samples = 1000

    def start(self):
        """Start timing a loop iteration."""
        self.start_time = time.time()

    def stop(self):
        """Stop timing and record duration."""
        if self.start_time is None:
            return

        duration = time.time() - self.start_time
        self.durations.append(duration)

        # Keep only recent samples
        if len(self.durations) > self.max_samples:
            self.durations = self.durations[-self.max_samples :]

        self.start_time = None

    def get_stats(self) -> dict:
        """
        Get timing statistics.

        Returns:
            Dictionary with mean, max, min duration (ms)
        """
        if not self.durations:
            return {"mean_ms": 0.0, "max_ms": 0.0, "min_ms": 0.0, "count": 0}

        durations_ms = [d * 1000 for d in self.durations]

        return {
            "mean_ms": sum(durations_ms) / len(durations_ms),
            "max_ms": max(durations_ms),
            "min_ms": min(durations_ms),
            "count": len(durations_ms),
        }

    def print_stats(self):
        """Print timing statistics."""
        stats = self.get_stats()
        print(
            f"[{self.name}] Mean: {stats['mean_ms']:.2f} ms, "
            f"Max: {stats['max_ms']:.2f} ms, "
            f"Min: {stats['min_ms']:.2f} ms "
            f"(n={stats['count']})"
        )
