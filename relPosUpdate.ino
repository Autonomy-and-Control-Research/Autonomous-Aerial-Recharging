#include "MAVLink.h"
#include "HardwareSerial.h"

#define TX 14
#define RX 15

// MAVLink parameters
#define SYSID 1
#define COMPID 192
#define TARGETSYSID 1
#define TARGETCOMPID 0
#define BAUD 57600

HardwareSerial mavSerial(1);  // Use UART1

void sendPosOffset(float x, float y, float z);

void setup() {
  mavSerial.begin(BAUD, SERIAL_8N1, RX, TX);
}

void loop() {
  send_mavlink_position_offset(0, 0, -1.0);  // Ascend 1 meter
  heartbeat();
  delay(1000);
}

void heartbeat(){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_heartbeat_t heart = {};
  heart.custom_mode = 1;
  heart.autopilot = MAV_AUTOPILOT_INVALID;
  heart.base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
  heart.type = MAV_TYPE_ONBOARD_CONTROLLER;
  mavlink_msg_heartbeat_encode(SYSID, COMPID, &msg, &heart);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}

void send_mavlink_position_offset(float x, float y, float z) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_set_position_target_local_ned_t target = {};
  // target.target_system = TARGETSYSID;
  // target.target_component = TARGETCOMPID;
  target.coordinate_frame = MAV_FRAME_LOCAL_OFFSET_NED;
  target.type_mask = 0b0000111111111000;  // Ignore velocity, acceleration, yaw
  target.x = x;
  target.y = y;
  target.z = z;

  mavlink_msg_set_position_target_local_ned_encode(SYSID, COMPID, &msg, &target);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}
