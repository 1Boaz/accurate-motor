/// Temporary double P controller, simulates a second order algorithm
pub fn second_order(wanted_1: i32, measured_1: i32, wanted_2: i32, measured_2: i32, kp_1: f32, kp_2: f32) -> f32 {
	((wanted_1 - measured_1) as f32 * kp_1) + ((wanted_2 - measured_2) as f32 * kp_2)
}

/// Temporary P controller
pub fn first_order(wanted: i32, slots: i32, kp: f32) -> f32 {
	(wanted - slots) as f32 * kp
}