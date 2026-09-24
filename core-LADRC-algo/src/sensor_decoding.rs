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
#[derive(Debug, Clone, Copy, PartialEq)]
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
	fn new(slot_offset: i32, sensors_start_state: SensorsState) -> Self;

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
	fn new(slot_offset: i32, sensors_start_state: SensorsState) -> Self {
		Self {
			last_measure: sensors_start_state,
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

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn new_SensorsState() {
		let case = SensorsState::new(true, true);
		assert_eq!(case, SensorsState {sensor1: true, sensor2: true});

		let case = SensorsState::new(false, false);
		assert_eq!(case, SensorsState {sensor1: false, sensor2: false});

		let case = SensorsState::new(true, false);
		assert_eq!(case, SensorsState {sensor1: true, sensor2: false});
	}

	#[test]
	fn two_bits_SensorsState() {
		let case = SensorsState::new(true, false);
		assert_eq!(case.as_bits(), 2);

		let case = SensorsState::new(false, true);
		assert_eq!(case.as_bits(), 1);

		let case = SensorsState::new(false, false);
		assert_eq!(case.as_bits(), 0);

		let case = SensorsState::new(true, true);
		assert_eq!(case.as_bits(), 3);
	}
	
	#[test]
	fn new_SensorsPairState() {
		let sensors_0_offset = SensorsPairState::new(0, SensorsState::new(false, false));
		assert_eq!(sensors_0_offset, SensorsPairState {
			last_measure: SensorsState::new(false, false),
			slots: 0,
		});

		let sensors_0_offset = SensorsPairState::new(0, SensorsState { sensor1: false, sensor2: true });
		assert_eq!(sensors_0_offset, SensorsPairState {
			last_measure: SensorsState { sensor1: false, sensor2: true },
			slots: 0,
		});

		let sensors_positive_offset = SensorsPairState::new(50, SensorsState::new(false, false));
		assert_eq!(sensors_positive_offset, SensorsPairState {
			last_measure: SensorsState::new(false, false),
			slots: 50,
		});

		let sensors_negative_offset = SensorsPairState::new(-30, SensorsState::new(false, false));
		assert_eq!(sensors_negative_offset, SensorsPairState {
			last_measure: SensorsState::new(false, false),
			slots: -30,
		});
	}

	#[test]
	fn update_state_SensorsPairState() {
		// 'f' for forwards
		let mut sensors_f_step = SensorsPairState::new(0, SensorsState::new(false, false));
		sensors_f_step.update_state(SensorsState::new(true, false));
		assert_eq!(sensors_f_step, SensorsPairState::new(1, SensorsState::new(true, false)));

		// 'b' for backwards
		let mut sensors_b_step = SensorsPairState::new(0, SensorsState::new(true, false));
		sensors_b_step.update_state(SensorsState::new(false, false));
		assert_eq!(sensors_b_step, SensorsPairState::new(-1, SensorsState::new(false, false)));

		let mut sensors_fbf_step = SensorsPairState::new(0, SensorsState::new(false, false));
		assert_eq!((true, true), sensors_fbf_step.update_state(SensorsState::new(true, false)));
		assert_eq!((true, false), sensors_fbf_step.update_state(SensorsState::new(false, false)));
		assert_eq!((true, true), sensors_fbf_step.update_state(SensorsState::new(true, false)));
		assert_eq!(sensors_f_step, SensorsPairState::new(1, SensorsState::new(true, false)));

		let mut not_moved_step = SensorsPairState::new(0, SensorsState::new(false, false));
		assert_eq!((false, false), not_moved_step.update_state(SensorsState::new(false, false)));
		assert_eq!(0, not_moved_step.slots);

		let mut skipped_step = SensorsPairState::new(5, SensorsState::new(false, false));
		assert_eq!((false, false), skipped_step.update_state(SensorsState::new(true, true)));
		assert_eq!(5, skipped_step.slots);
	}

	#[test]
	fn reset_SensorsPairState() {
		let mut sensors_positive_offset = SensorsPairState::new(0, SensorsState::new(false, false));
		sensors_positive_offset.reset(5);
		assert_eq!(sensors_positive_offset.slots, 5);

		let mut sensors_negative_offset = SensorsPairState::new(0, SensorsState::new(false, false));
		sensors_negative_offset.reset(-5);
		assert_eq!(sensors_negative_offset.slots, -5);
	}
}