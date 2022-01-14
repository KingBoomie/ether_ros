Header header
uint8 slave_id

uint16 tx1_control_word
uint8  tx1_mode_of_operation
uint32 tx1_target_position

uint16 tx2_control_word
uint8  tx2_mode_of_operation
uint32 tx2_target_position
uint32 tx2_velocity
uint32 tx2_target_acceleration
uint32 tx2_target_deceleration

uint16 tx3_control_word
uint8  tx3_mode_of_operation
uint32 tx3_target_acceleration
uint32 tx3_target_deceleration
uint32 tx3_target_velocity

int8 READONLY=0
int8 TX1=1
int8 TX2=2
int8 TX3=3
int8 txtype

