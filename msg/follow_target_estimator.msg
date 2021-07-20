uint64 timestamp 			# time since system start (microseconds)
uint64 last_filter_reset_timestamp	# time of last filter reset (microseconds)

bool valid		# True if estimator states are okay to be used
bool stale		# True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate.

float64 lat_est		# Estimated target latitude
float64 lon_est		# Estimated target longitude
float32 alt_est		# Estimated target altitude

float32 x_est 		# Estimated target NED position (m)
float32 y_est 		# Estimated target NED position (m)
float32 z_est 		# Estimated target NED position (m)

float32 vx_est		# Estimated target NED velocity (m/s)
float32 vy_est		# Estimated target NED velocity (m/s)
float32 vz_est		# Estimated target NED velocity (m/s)

float32 ax_est		# Estimated target NED acceleration (m^2/s)
float32 ay_est		# Estimated target NED acceleration (m^2/s)
float32 az_est		# Estimated target NED acceleration (m^2/s)
