pub mod nrf;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmChannel {
    Event = 0,
    Cpu,
    NumAlarmChannels,
}
