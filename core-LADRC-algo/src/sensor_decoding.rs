/// Represents the raw digital state of a dual-sensor optical encoder pair.
///
/// # Examples
///
/// ```
/// use core_LADRC_algo::sensor_decoding::SensorsState;
///
/// let state = SensorsState::new(true, false);
/// assert!(state.sensor1());
/// assert!(!state.sensor2());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorsState {
	/// State of the first optical sensor (`true` for high, `false` for low).
	sensor1: bool,
	/// State of the second optical sensor (`true` for high, `false` for low).
	sensor2: bool,
}

impl SensorsState {
	/// Creates a new [`SensorsState`] from raw sensor readings.
	pub fn new(sensor1: bool, sensor2: bool) -> Self {
		Self { sensor1, sensor2 }
	}

	/// Returns the state of the first optical sensor.
	pub fn sensor1(&self) -> bool {
		self.sensor1
	}

	/// Returns the state of the second optical sensor.
	pub fn sensor2(&self) -> bool {
		self.sensor2
	}
}

/// A fixed-size ring buffer maintaining recent encoder states.
#[derive(Debug, Clone, Copy)]
pub struct SensorsPairState {
	/// Stores the 4 most recent sensor measurements.
	past_4_measures: [SensorsState; 4],
	/// Points to the most recently written slot in `past_4_measures`.
	current_index: usize,
	/// Saves the amount of slots counted since initialization(times by ratio is degrees).
	/// Counted counter-clockwise positive.
	slots: i32,
}

/// Interface for storing encoder state history and decoding rotational movement.
pub trait SensorsPair {
	/// Constructs a new encoder tracker initialized with default state values.
	/// Using the [`slot_offset`] parameter an offset can be given.
	///
	/// # Examples
	///
	/// ```
	/// use core_LADRC_algo::sensor_decoding::{SensorsPairState, SensorsPair};
	///
	/// let encoder = SensorsPairState::new(0);
	/// ```
	fn new(slot_offset: i32) -> Self;

	/// Records a new [`SensorsState`] updates the slots counter and returns the direction delta (first bool is true for any movement, second is true only for CCW movement).
	///
	/// # Examples
	///
	/// ```
	/// use core_LADRC_algo::sensor_decoding::{SensorsPair, SensorsPairState, SensorsState};
	///
	/// let mut encoder = SensorsPairState::new(0);
	/// let new_state = SensorsState::new(true, false);
	/// let _delta = encoder.update_state(new_state);
	/// ```
	fn update_state(&mut self, new_state: SensorsState) -> (bool, bool);

	/// Resets the slots counter, intended for drift reset and useful with a limit switch.
	/// Using the [`slot_offset`] parameter an offset can be given.
	///
	/// # Examples
	///
	/// ```
	/// use core_LADRC_algo::sensor_decoding::{SensorsPairState, SensorsPair};
	///
	/// let mut encoder = SensorsPairState::new(0);
	///
	/// encoder.reset(36);
	/// ```
	fn reset(&mut self, slots_offset: i32);
}

impl SensorsPair for SensorsPairState {
	fn new(slot_offset: i32) -> Self {
		Self {
			past_4_measures: [SensorsState { sensor1: false, sensor2: true }; 4],
			current_index: 0,
			slots: slot_offset,
		}
	}

	fn update_state(&mut self, new_state: SensorsState) -> (bool, bool) {
		// Advance index with bitwise wrap (equivalent to % 4 for power-of-2 size 4)
		self.current_index = (self.current_index + 1) & 3;
		self.past_4_measures[self.current_index] = new_state;
		todo!("Complete the direction sensing by comparing the states");
	}

	fn reset(&mut self, slots_offset: i32) {
		self.slots = slots_offset;
	}
}