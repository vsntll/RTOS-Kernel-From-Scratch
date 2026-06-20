#ifndef XRCE_MSGS_H
#define XRCE_MSGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "xrce/cdr.h"

/* CDR (de)serializers for a small, deliberately-real subset of standard
 * ROS2 message types -- field layout matches the actual .msg definitions
 * (std_msgs/Int32, std_msgs/String, sensor_msgs/Imu) exactly, including
 * field order, so a real ROS2 subscriber can deserialize what this writes
 * once it's flowing through Phase 4's agent link. Each *_encode() call
 * writes the 4-byte CDR header itself -- callers pass a fresh writer, not
 * one that's already had xrce_cdr_write_header() called on it. */

#define XRCE_MSGS_FRAME_ID_MAX 64

typedef struct {
    int32_t data;
} std_msgs_Int32;

bool std_msgs_Int32_encode(const std_msgs_Int32 *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool std_msgs_Int32_decode(const uint8_t *buf, size_t len, std_msgs_Int32 *out);

#define XRCE_MSGS_STRING_MAX 256

typedef struct {
    char data[XRCE_MSGS_STRING_MAX];
} std_msgs_String;

bool std_msgs_String_encode(const std_msgs_String *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool std_msgs_String_decode(const uint8_t *buf, size_t len, std_msgs_String *out);

typedef struct {
    double x, y, z, w;
} geometry_msgs_Quaternion;

typedef struct {
    double x, y, z;
} geometry_msgs_Vector3;

typedef struct {
    int32_t sec;
    uint32_t nanosec;
} builtin_interfaces_Time;

typedef struct {
    builtin_interfaces_Time stamp;
    char frame_id[XRCE_MSGS_FRAME_ID_MAX];
} std_msgs_Header;

typedef struct {
    std_msgs_Header header;
    geometry_msgs_Quaternion orientation;
    double orientation_covariance[9];
    geometry_msgs_Vector3 angular_velocity;
    double angular_velocity_covariance[9];
    geometry_msgs_Vector3 linear_acceleration;
    double linear_acceleration_covariance[9];
} sensor_msgs_Imu;

bool sensor_msgs_Imu_encode(const sensor_msgs_Imu *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool sensor_msgs_Imu_decode(const uint8_t *buf, size_t len, sensor_msgs_Imu *out);

/* std_srvs/srv/Trigger -- Phase 7b's demo service ("trigger a self-test").
 * Request has no fields at all (still gets its own 4-byte CDR header, same
 * as every other message here -- an empty struct isn't a zero-byte wire
 * message). Response is `bool success; string message`, field order
 * ground-truthed against `ros2 interface show std_srvs/srv/Trigger`. */
typedef struct {
    int _unused; /* C forbids an empty struct; never read. */
} std_srvs_Trigger_Request;

bool std_srvs_Trigger_Request_encode(const std_srvs_Trigger_Request *msg, uint8_t *buf, size_t cap,
                                      size_t *out_len);
bool std_srvs_Trigger_Request_decode(const uint8_t *buf, size_t len, std_srvs_Trigger_Request *out);

typedef struct {
    bool success;
    char message[XRCE_MSGS_STRING_MAX];
} std_srvs_Trigger_Response;

bool std_srvs_Trigger_Response_encode(const std_srvs_Trigger_Response *msg, uint8_t *buf, size_t cap,
                                       size_t *out_len);
bool std_srvs_Trigger_Response_decode(const uint8_t *buf, size_t len, std_srvs_Trigger_Response *out);

#endif
