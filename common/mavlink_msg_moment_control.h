#pragma once
// MESSAGE MOMENT_CONTROL PACKING

#define MAVLINK_MSG_ID_MOMENT_CONTROL 339

MAVPACKED(
typedef struct __mavlink_moment_control_t {
 float roll; /*<  Roll Moment; -1..1*/
 float pitch; /*<  Pitch Moment; -1..1*/
 float yaw; /*<  Yaw Moment; -1..1*/
 float thrust; /*<  Thrust; 0..1*/
}) mavlink_moment_control_t;

#define MAVLINK_MSG_ID_MOMENT_CONTROL_LEN 16
#define MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN 16
#define MAVLINK_MSG_ID_339_LEN 16
#define MAVLINK_MSG_ID_339_MIN_LEN 16

#define MAVLINK_MSG_ID_MOMENT_CONTROL_CRC 118
#define MAVLINK_MSG_ID_339_CRC 118



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOMENT_CONTROL { \
    339, \
    "MOMENT_CONTROL", \
    4, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_moment_control_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_moment_control_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_moment_control_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_moment_control_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOMENT_CONTROL { \
    "MOMENT_CONTROL", \
    4, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_moment_control_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_moment_control_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_moment_control_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_moment_control_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a moment_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll  Roll Moment; -1..1
 * @param pitch  Pitch Moment; -1..1
 * @param yaw  Yaw Moment; -1..1
 * @param thrust  Thrust; 0..1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_moment_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOMENT_CONTROL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN);
#else
    mavlink_moment_control_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOMENT_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
}

/**
 * @brief Pack a moment_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll  Roll Moment; -1..1
 * @param pitch  Pitch Moment; -1..1
 * @param yaw  Yaw Moment; -1..1
 * @param thrust  Thrust; 0..1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_moment_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOMENT_CONTROL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN);
#else
    mavlink_moment_control_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOMENT_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
}

/**
 * @brief Encode a moment_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param moment_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_moment_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_moment_control_t* moment_control)
{
    return mavlink_msg_moment_control_pack(system_id, component_id, msg, moment_control->roll, moment_control->pitch, moment_control->yaw, moment_control->thrust);
}

/**
 * @brief Encode a moment_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param moment_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_moment_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_moment_control_t* moment_control)
{
    return mavlink_msg_moment_control_pack_chan(system_id, component_id, chan, msg, moment_control->roll, moment_control->pitch, moment_control->yaw, moment_control->thrust);
}

/**
 * @brief Send a moment_control message
 * @param chan MAVLink channel to send the message
 *
 * @param roll  Roll Moment; -1..1
 * @param pitch  Pitch Moment; -1..1
 * @param yaw  Yaw Moment; -1..1
 * @param thrust  Thrust; 0..1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_moment_control_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOMENT_CONTROL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOMENT_CONTROL, buf, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
#else
    mavlink_moment_control_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOMENT_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
#endif
}

/**
 * @brief Send a moment_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_moment_control_send_struct(mavlink_channel_t chan, const mavlink_moment_control_t* moment_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_moment_control_send(chan, moment_control->roll, moment_control->pitch, moment_control->yaw, moment_control->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOMENT_CONTROL, (const char *)moment_control, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOMENT_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_moment_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOMENT_CONTROL, buf, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
#else
    mavlink_moment_control_t *packet = (mavlink_moment_control_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOMENT_CONTROL, (const char *)packet, MAVLINK_MSG_ID_MOMENT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN, MAVLINK_MSG_ID_MOMENT_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE MOMENT_CONTROL UNPACKING


/**
 * @brief Get field roll from moment_control message
 *
 * @return  Roll Moment; -1..1
 */
static inline float mavlink_msg_moment_control_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from moment_control message
 *
 * @return  Pitch Moment; -1..1
 */
static inline float mavlink_msg_moment_control_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from moment_control message
 *
 * @return  Yaw Moment; -1..1
 */
static inline float mavlink_msg_moment_control_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust from moment_control message
 *
 * @return  Thrust; 0..1
 */
static inline float mavlink_msg_moment_control_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a moment_control message into a struct
 *
 * @param msg The message to decode
 * @param moment_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_moment_control_decode(const mavlink_message_t* msg, mavlink_moment_control_t* moment_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    moment_control->roll = mavlink_msg_moment_control_get_roll(msg);
    moment_control->pitch = mavlink_msg_moment_control_get_pitch(msg);
    moment_control->yaw = mavlink_msg_moment_control_get_yaw(msg);
    moment_control->thrust = mavlink_msg_moment_control_get_thrust(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOMENT_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_MOMENT_CONTROL_LEN;
        memset(moment_control, 0, MAVLINK_MSG_ID_MOMENT_CONTROL_LEN);
    memcpy(moment_control, _MAV_PAYLOAD(msg), len);
#endif
}
