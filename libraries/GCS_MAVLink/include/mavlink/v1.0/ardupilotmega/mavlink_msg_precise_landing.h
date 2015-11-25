// MESSAGE PRECISE_LANDING PACKING

#define MAVLINK_MSG_ID_PRECISE_LANDING 226

typedef struct __mavlink_precise_landing_t
{
 float target_z_distance; /*< z distance above target ground*/
 float bf_offset_x; /*< x offset of target to body frame*/
 float bf_offset_y; /*< y offset of target to body frame*/
 uint8_t num_of_targets; /*< number of detected targets*/
} mavlink_precise_landing_t;

#define MAVLINK_MSG_ID_PRECISE_LANDING_LEN 13
#define MAVLINK_MSG_ID_226_LEN 13

#define MAVLINK_MSG_ID_PRECISE_LANDING_CRC 177
#define MAVLINK_MSG_ID_226_CRC 177



#define MAVLINK_MESSAGE_INFO_PRECISE_LANDING { \
	"PRECISE_LANDING", \
	4, \
	{  { "target_z_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_precise_landing_t, target_z_distance) }, \
         { "bf_offset_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_precise_landing_t, bf_offset_x) }, \
         { "bf_offset_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_precise_landing_t, bf_offset_y) }, \
         { "num_of_targets", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_precise_landing_t, num_of_targets) }, \
         } \
}


/**
 * @brief Pack a precise_landing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param num_of_targets number of detected targets
 * @param target_z_distance z distance above target ground
 * @param bf_offset_x x offset of target to body frame
 * @param bf_offset_y y offset of target to body frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_precise_landing_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t num_of_targets, float target_z_distance, float bf_offset_x, float bf_offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PRECISE_LANDING_LEN];
	_mav_put_float(buf, 0, target_z_distance);
	_mav_put_float(buf, 4, bf_offset_x);
	_mav_put_float(buf, 8, bf_offset_y);
	_mav_put_uint8_t(buf, 12, num_of_targets);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#else
	mavlink_precise_landing_t packet;
	packet.target_z_distance = target_z_distance;
	packet.bf_offset_x = bf_offset_x;
	packet.bf_offset_y = bf_offset_y;
	packet.num_of_targets = num_of_targets;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PRECISE_LANDING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
}

/**
 * @brief Pack a precise_landing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param num_of_targets number of detected targets
 * @param target_z_distance z distance above target ground
 * @param bf_offset_x x offset of target to body frame
 * @param bf_offset_y y offset of target to body frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_precise_landing_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t num_of_targets,float target_z_distance,float bf_offset_x,float bf_offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PRECISE_LANDING_LEN];
	_mav_put_float(buf, 0, target_z_distance);
	_mav_put_float(buf, 4, bf_offset_x);
	_mav_put_float(buf, 8, bf_offset_y);
	_mav_put_uint8_t(buf, 12, num_of_targets);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#else
	mavlink_precise_landing_t packet;
	packet.target_z_distance = target_z_distance;
	packet.bf_offset_x = bf_offset_x;
	packet.bf_offset_y = bf_offset_y;
	packet.num_of_targets = num_of_targets;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PRECISE_LANDING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
}

/**
 * @brief Encode a precise_landing struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param precise_landing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_precise_landing_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_precise_landing_t* precise_landing)
{
	return mavlink_msg_precise_landing_pack(system_id, component_id, msg, precise_landing->num_of_targets, precise_landing->target_z_distance, precise_landing->bf_offset_x, precise_landing->bf_offset_y);
}

/**
 * @brief Encode a precise_landing struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param precise_landing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_precise_landing_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_precise_landing_t* precise_landing)
{
	return mavlink_msg_precise_landing_pack_chan(system_id, component_id, chan, msg, precise_landing->num_of_targets, precise_landing->target_z_distance, precise_landing->bf_offset_x, precise_landing->bf_offset_y);
}

/**
 * @brief Send a precise_landing message
 * @param chan MAVLink channel to send the message
 *
 * @param num_of_targets number of detected targets
 * @param target_z_distance z distance above target ground
 * @param bf_offset_x x offset of target to body frame
 * @param bf_offset_y y offset of target to body frame
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_precise_landing_send(mavlink_channel_t chan, uint8_t num_of_targets, float target_z_distance, float bf_offset_x, float bf_offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PRECISE_LANDING_LEN];
	_mav_put_float(buf, 0, target_z_distance);
	_mav_put_float(buf, 4, bf_offset_x);
	_mav_put_float(buf, 8, bf_offset_y);
	_mav_put_uint8_t(buf, 12, num_of_targets);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
#else
	mavlink_precise_landing_t packet;
	packet.target_z_distance = target_z_distance;
	packet.bf_offset_x = bf_offset_x;
	packet.bf_offset_y = bf_offset_y;
	packet.num_of_targets = num_of_targets;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, (const char *)&packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, (const char *)&packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PRECISE_LANDING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_precise_landing_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t num_of_targets, float target_z_distance, float bf_offset_x, float bf_offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, target_z_distance);
	_mav_put_float(buf, 4, bf_offset_x);
	_mav_put_float(buf, 8, bf_offset_y);
	_mav_put_uint8_t(buf, 12, num_of_targets);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, buf, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
#else
	mavlink_precise_landing_t *packet = (mavlink_precise_landing_t *)msgbuf;
	packet->target_z_distance = target_z_distance;
	packet->bf_offset_x = bf_offset_x;
	packet->bf_offset_y = bf_offset_y;
	packet->num_of_targets = num_of_targets;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, (const char *)packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN, MAVLINK_MSG_ID_PRECISE_LANDING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRECISE_LANDING, (const char *)packet, MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PRECISE_LANDING UNPACKING


/**
 * @brief Get field num_of_targets from precise_landing message
 *
 * @return number of detected targets
 */
static inline uint8_t mavlink_msg_precise_landing_get_num_of_targets(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_z_distance from precise_landing message
 *
 * @return z distance above target ground
 */
static inline float mavlink_msg_precise_landing_get_target_z_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field bf_offset_x from precise_landing message
 *
 * @return x offset of target to body frame
 */
static inline float mavlink_msg_precise_landing_get_bf_offset_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field bf_offset_y from precise_landing message
 *
 * @return y offset of target to body frame
 */
static inline float mavlink_msg_precise_landing_get_bf_offset_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a precise_landing message into a struct
 *
 * @param msg The message to decode
 * @param precise_landing C-struct to decode the message contents into
 */
static inline void mavlink_msg_precise_landing_decode(const mavlink_message_t* msg, mavlink_precise_landing_t* precise_landing)
{
#if MAVLINK_NEED_BYTE_SWAP
	precise_landing->target_z_distance = mavlink_msg_precise_landing_get_target_z_distance(msg);
	precise_landing->bf_offset_x = mavlink_msg_precise_landing_get_bf_offset_x(msg);
	precise_landing->bf_offset_y = mavlink_msg_precise_landing_get_bf_offset_y(msg);
	precise_landing->num_of_targets = mavlink_msg_precise_landing_get_num_of_targets(msg);
#else
	memcpy(precise_landing, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PRECISE_LANDING_LEN);
#endif
}
