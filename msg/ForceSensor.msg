# FORCE_SENSOR message data

uint64 timestamp		# time since system start (microseconds)
uint32 device_id

float32 force
int32 force_int
char[17] force_string
int32 raw_adc

uint8 lp_filter
uint8 avg_filter
uint8 ema_filter
float32 gap_value
uint8 i2c_address
uint8 firmware_version
