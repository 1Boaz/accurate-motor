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

	/// Packs the two sensor readings into a 2-bit state.
	///
	/// Sensor 1 is the most significant bit.
	#[inline]
	fn as_bits(&self) -> u8 {
		((self.sensor1 as u8) << 1) | (self.sensor2 as u8)
	}
}

/// A struct to save the last encoder state for direction comparison and slots counter.
#[derive(Debug, Clone, Copy)]
pub struct SensorsPairState {
	/// Stores the most recent sensor measurement.
	last_measure: SensorsState,
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
	/// let (is_moving, is_CCW) = encoder.update_state(new_state);
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
			last_measure: SensorsState { sensor1: false, sensor2: true },
			slots: slot_offset,
		}
	}

	fn update_state(&mut self, new_state: SensorsState) -> (bool, bool) {
		// Quadrature transition lookup table.
		// Index = (previous_state << 2) | current_state
		//
		// +1 = CCW
		// -1 = CW
		//  0 = unchanged or invalid transition
		const LUT: [i8; 16] = [
			0, -1,  1,  0,
			1,  0,  0, -1,
			-1,  0,  0,  1,
			0,  1, -1,  0,
		];

		let previous = self.last_measure.as_bits();
		let current = new_state.as_bits();

		let index = ((previous << 2) | current) as usize;
		let delta = LUT[index];

		self.last_measure = new_state;

		if delta == 0 {
			return (false, false);
		}

		self.slots = self.slots.wrapping_add(delta as i32);

		(true, delta > 0)
	}

	fn reset(&mut self, slots_offset: i32) {
		self.slots = slots_offset;
	}
}