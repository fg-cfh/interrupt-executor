use fugit::{Instant, NanosDurationU64};

pub type SyntonizedInstant = Instant<u64, 1, 1_000_000_000>;
pub type SyntonizedDuration = NanosDurationU64;

pub trait RadioTimerApi: Sized {
    fn now() -> SyntonizedInstant;

    /// Waits until the given instant, then wakes the current task.
    ///
    /// Note: This method induces considerable latency and jitter as there may
    ///       be an arbitrary delay between waking the task and the task
    ///       executing. For more precisely timed alarms, use one of the
    ///       hardware-backed methods
    fn wait_until(instant: SyntonizedInstant) -> impl Future<Output = ()>;
}
