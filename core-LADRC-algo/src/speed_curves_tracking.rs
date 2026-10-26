/// A struct which represents an interval on a graph
#[derive(Copy, Clone)]
pub struct Interval<'a> {
	/// Which x of this function interval should it start with
	local_x_start: i32,
	/// How long before continuing to the next curve
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
	/// But every closure that takes an i32 and returns an i32 is valid
	function: &'a dyn Fn(i32) -> i32,

	/// Functions like a linked list, connects to the next node
	next: Option<&'a Interval<'a>>
}

impl Interval<'_> {
	/// Creates a new [`Interval`] instance with the given values
	pub fn new<'a>(func: &'a dyn Fn(i32) -> i32, x_start: i32, length: u32, next: Option<&'a Interval<'a>>) -> Interval<'a> {
		Interval {
			function: func,
			local_x_start: x_start,
			length,
			next,
		}
	}
}
