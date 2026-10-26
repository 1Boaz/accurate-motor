/// A struct which represents an interval on a graph
#[derive(Copy, Clone)]
pub struct Interval<'a> {
	/// Which x of this function interval should it start with
	local_x_start: u32,
	/// How long before continuing to the next curve(inclusive)
	length: u32,
	/// Stores the function that's calculating the needed speed
	/// The expected format is:
	/// f(x) = something with x
	/// Some common options are:
	/// ###### Constant speed, or a set speed which doesn't change
	/// f(_) = constant number
	/// ###### Linear acceleration/deceleration
	/// f(x) = mx + b
	///
	/// But every closure that takes an u32 and returns an u32 is valid
	function: &'a dyn Fn(u32) -> u32,

	/// Functions like a linked list, connects to the next node
	next: Option<&'a Interval<'a>>
}

impl Interval<'_> {
	/// Creates a new [`Interval`] instance with the given values
	///
	/// # Examples
	///
	/// Creating a single standalone interval:
	///
	/// ```
	/// use core_LADRC_algo::speed_curves_tracking::Interval;
	/// let func = |x| x + 5;
	/// let interval = Interval::new(&func, 0, 10, None);
	/// ```
	///
	/// Chaining intervals together like a linked list:
	///
	/// ```
	/// use core_LADRC_algo::speed_curves_tracking::Interval;
	/// let f2 = |_| 42;
	/// let interval_two = Interval::new(&f2, 0, 5, None);
	///
	/// let f1 = |x| 2 * x;
	/// let interval_one = Interval::new(&f1, 0, 5, Some(&interval_two));
	/// ```
	pub fn new<'a>(func: &'a dyn Fn(u32) -> u32, x_start: u32, length: u32, next: Option<&'a Interval<'a>>) -> Interval<'a> {
		Interval {
			function: func,
			local_x_start: x_start,
			length,
			next,
		}
	}
}

/// Struct to hold all the intervals and keep state
pub struct AccelerationCurve<'a> {
	/// State of the current x on the acceleration curve(globally)
	global_x: u32,
	/// State of the current x on the current interval
	local_x: u32,
	/// Link to the first [`Interval`]
	intervals: Interval<'a>,
}

impl<'a> AccelerationCurve<'a> {
	/// Creates a new [`AccelerationCurve`] instance with the given intervals, should be given as a linked list
	///
	/// # Examples
	///
	/// ```
	/// use core_LADRC_algo::speed_curves_tracking::{AccelerationCurve, Interval};
	/// let func = |x| x;
	/// let interval = Interval::new(&func, 0, 5, None);
	/// let curve = AccelerationCurve::new(interval);
	/// ```
	pub fn new(intervals: Interval) -> AccelerationCurve {
		AccelerationCurve {
			global_x: 0,
			local_x: 0,
			intervals
		}
	}

	/// Steps one x on the acceleration curve, and returns the calculated value
	///
	/// # Errors
	///
	/// Returns an `Err("Reached end of the curve")` when the curve exceeds the final
	/// interval's length and has no subsequent interval to advance to.
	///
	/// # Examples
	///
	/// ### Stepping through a single interval:
	/// ```
	/// use core_LADRC_algo::speed_curves_tracking::{AccelerationCurve, Interval};
	/// let func = |x| 2 * x; // f(x) = 2x
	/// let interval = Interval::new(&func, 0, 2, None);
	/// let mut curve = AccelerationCurve::new(interval);
	///
	/// assert_eq!(curve.step(), Ok(2)); // x = 0 + 1 => 2 * 1
	/// assert_eq!(curve.step(), Ok(4)); // x = 0 + 2 => 2 * 2
	/// assert_eq!(curve.step(), Err("Reached end of the curve"));
	/// ```
	///
	/// ### Stepping across chained intervals:
	/// ```
	/// use core_LADRC_algo::speed_curves_tracking::{AccelerationCurve, Interval};
	/// let f2 = |_| 50; // Constant speed
	/// let int2 = Interval::new(&f2, 0, 1, None);
	///
	/// let f1 = |x| x * 10; // Linear speed
	/// let int1 = Interval::new(&f1, 0, 2, Some(&int2));
	///
	/// let mut curve = AccelerationCurve::new(int1);
	///
	/// // Steps in first interval
	/// assert_eq!(curve.step(), Ok(10));
	/// assert_eq!(curve.step(), Ok(20));
	///
	/// // Automatically transitions to second interval
	/// assert_eq!(curve.step(), Ok(50));
	///
	/// // End reached
	/// assert_eq!(curve.step(), Err("Reached end of the curve"));
	/// ```
	pub fn step(&mut self) -> Result<u32, &str> {
		self.local_x += 1;
		self.global_x += 1;

		// Checks if to advance the interval to the next one
		if self.intervals.length < self.local_x {
			if self.intervals.next.is_none() {
				return Err("Reached end of the curve")
			}
			// Safe because of the is_none check above
			self.intervals = *self.intervals.next.unwrap();
			self.local_x = 1;
		}

		let function_x = self.intervals.local_x_start + self.local_x;
		Ok((self.intervals.function)(function_x))
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn test_single_interval_constant() {
		let func = |_x| 42;
		let interval = Interval::new(&func, 0, 3, None);
		let mut curve = AccelerationCurve::new(interval);

		assert_eq!(curve.step(), Ok(42));
		assert_eq!(curve.step(), Ok(42));
		assert_eq!(curve.step(), Ok(42));
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_single_interval_linear() {
		// f(x) = 2x + 1
		let func = |x| 2 * x + 1;
		let interval = Interval::new(&func, 0, 4, None);
		let mut curve = AccelerationCurve::new(interval);

		// local_x starts at 1, 2, 3, 4
		assert_eq!(curve.step(), Ok(3)); // 2*1 + 1
		assert_eq!(curve.step(), Ok(5)); // 2*2 + 1
		assert_eq!(curve.step(), Ok(7)); // 2*3 + 1
		assert_eq!(curve.step(), Ok(9)); // 2*4 + 1
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_local_x_start_offset() {
		// f(x) = x, with local_x_start = 5
		let func = |x| x;
		let interval = Interval::new(&func, 5, 2, None);
		let mut curve = AccelerationCurve::new(interval);

		assert_eq!(curve.step(), Ok(6)); // 5 + 1
		assert_eq!(curve.step(), Ok(7)); // 5 + 2
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_chained_intervals() {
		// Interval 1: length 2, f(x) = x
		// Interval 2: length 2, f(x) = x + 10
		let f1 = |x| x;
		let f2 = |x| x + 10;
		let int2 = Interval::new(&f2, 0, 2, None);
		let int1 = Interval::new(&f1, 0, 2, Some(&int2));
		let mut curve = AccelerationCurve::new(int1);

		// Interval 1 steps
		assert_eq!(curve.step(), Ok(1));
		assert_eq!(curve.global_x, 1);
		assert_eq!(curve.local_x, 1);

		assert_eq!(curve.step(), Ok(2));
		assert_eq!(curve.global_x, 2);
		assert_eq!(curve.local_x, 2);

		// Interval 2 steps
		assert_eq!(curve.step(), Ok(11));
		assert_eq!(curve.global_x, 3);
		assert_eq!(curve.local_x, 1);

		assert_eq!(curve.step(), Ok(12));
		assert_eq!(curve.global_x, 4);
		assert_eq!(curve.local_x, 2);

		// End of curve
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_multiple_chained_intervals() {
		let f1 = |_x| 100;
		let f2 = |_x| 200;
		let f3 = |_x| 300;

		let int3 = Interval::new(&f3, 0, 1, None);
		let int2 = Interval::new(&f2, 0, 1, Some(&int3));
		let int1 = Interval::new(&f1, 0, 1, Some(&int2));

		let mut curve = AccelerationCurve::new(int1);

		assert_eq!(curve.step(), Ok(100));
		assert_eq!(curve.step(), Ok(200));
		assert_eq!(curve.step(), Ok(300));
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_zero_length_interval() {
		let func = |_x| 1;
		let interval = Interval::new(&func, 0, 0, None);
		let mut curve = AccelerationCurve::new(interval);

		// A length of 0 should immediately hit the end on the first step
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}

	#[test]
	fn test_repeated_calls_after_end() {
		let func = |_x| 10;
		let interval = Interval::new(&func, 0, 1, None);
		let mut curve = AccelerationCurve::new(interval);

		assert_eq!(curve.step(), Ok(10));
		assert_eq!(curve.step(), Err("Reached end of the curve"));
		assert_eq!(curve.step(), Err("Reached end of the curve"));
	}
}