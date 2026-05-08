std_msgs/Header header

uint8 DISABLED=0
uint8 SEARCH=1
uint8 ACQUIRE=2
uint8 TRACK=3
uint8 RECOVER=4
uint8 LIMITED_LOST=5

uint8 SOURCE_NONE=0
uint8 SOURCE_WIDE=1
uint8 SOURCE_ZOOM=2

uint8 state
uint8 source
string assist_mode
float32 confidence
float32 error_x
float32 error_y
float32 gimbal_yaw_cmd
float32 gimbal_pitch_cmd
float32 aircraft_yaw_rate_cmd
float32 aircraft_x_cmd
float32 aircraft_y_cmd
bool gimbal_limit_active
bool flight_assist_active
string limit_reason
