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

/* Phase 14: drives a simulated vehicle's diff-drive plugin in Gazebo
 * (geometry_msgs/msg/Twist over /model/<name>/cmd_vel, bridged by
 * ros_gz_bridge -- see xrce/docs/design.md's Phase 14 section). Just two
 * Vector3s, reusing the write_vec3()/read_vec3() helpers Imu's
 * angular_velocity/linear_acceleration fields already use. */
typedef struct {
    geometry_msgs_Vector3 linear;
    geometry_msgs_Vector3 angular;
} geometry_msgs_Twist;

bool geometry_msgs_Twist_encode(const geometry_msgs_Twist *msg, uint8_t *buf, size_t cap,
                                 size_t *out_len);
bool geometry_msgs_Twist_decode(const uint8_t *buf, size_t len, geometry_msgs_Twist *out);

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

/* Phase 7b actions: example_interfaces/action/Fibonacci (goal `int32
 * order`, result/feedback `int32[] sequence`) plus the synthesized
 * SendGoal/GetResult/FeedbackMessage wrapper types real rclcpp/rclpy
 * actions use under the hood -- every field layout here is copied from the
 * REAL generated headers installed under
 * /opt/ros/jazzy/include/example_interfaces/.../detail/fibonacci__struct.h
 * and .../unique_identifier_msgs/.../uuid__struct.h, not guessed. A goal_id
 * is a bare `uint8_t[16]` (no length prefix -- it's a fixed array, not a
 * CDR sequence, per unique_identifier_msgs/msg/UUID). */
#define XRCE_MSGS_UUID_SIZE 16
#define XRCE_MSGS_SEQ_I32_MAX 16

typedef struct {
    int32_t order;
} example_interfaces_Fibonacci_Goal;

typedef struct {
    int32_t sequence[XRCE_MSGS_SEQ_I32_MAX];
    uint32_t sequence_count;
} example_interfaces_Fibonacci_Result;

typedef example_interfaces_Fibonacci_Result example_interfaces_Fibonacci_Feedback;

typedef struct {
    uint8_t goal_id[XRCE_MSGS_UUID_SIZE];
    int32_t order;
} example_interfaces_Fibonacci_SendGoal_Request;

bool example_interfaces_Fibonacci_SendGoal_Request_encode(
    const example_interfaces_Fibonacci_SendGoal_Request *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool example_interfaces_Fibonacci_SendGoal_Request_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_SendGoal_Request *out);

typedef struct {
    bool accepted;
    builtin_interfaces_Time stamp;
} example_interfaces_Fibonacci_SendGoal_Response;

bool example_interfaces_Fibonacci_SendGoal_Response_encode(
    const example_interfaces_Fibonacci_SendGoal_Response *msg, uint8_t *buf, size_t cap,
    size_t *out_len);
bool example_interfaces_Fibonacci_SendGoal_Response_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_SendGoal_Response *out);

typedef struct {
    uint8_t goal_id[XRCE_MSGS_UUID_SIZE];
} example_interfaces_Fibonacci_GetResult_Request;

bool example_interfaces_Fibonacci_GetResult_Request_encode(
    const example_interfaces_Fibonacci_GetResult_Request *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool example_interfaces_Fibonacci_GetResult_Request_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_GetResult_Request *out);

typedef struct {
    int8_t status;
    example_interfaces_Fibonacci_Result result;
} example_interfaces_Fibonacci_GetResult_Response;

bool example_interfaces_Fibonacci_GetResult_Response_encode(
    const example_interfaces_Fibonacci_GetResult_Response *msg, uint8_t *buf, size_t cap,
    size_t *out_len);
bool example_interfaces_Fibonacci_GetResult_Response_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_GetResult_Response *out);

typedef struct {
    uint8_t goal_id[XRCE_MSGS_UUID_SIZE];
    example_interfaces_Fibonacci_Feedback feedback;
} example_interfaces_Fibonacci_FeedbackMessage;

bool example_interfaces_Fibonacci_FeedbackMessage_encode(
    const example_interfaces_Fibonacci_FeedbackMessage *msg, uint8_t *buf, size_t cap, size_t *out_len);
bool example_interfaces_Fibonacci_FeedbackMessage_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_FeedbackMessage *out);

/* action_msgs/msg/GoalInfo, GoalStatus(Array), and srv/CancelGoal -- real,
 * already-installed standard interfaces (not synthesized per-action),
 * field layouts likewise copied from the real generated headers under
 * /opt/ros/jazzy/include/action_msgs, .../detail/ struct headers. */
#define XRCE_MSGS_GOAL_STATUS_MAX 8

typedef struct {
    uint8_t goal_id[XRCE_MSGS_UUID_SIZE];
    builtin_interfaces_Time stamp;
} action_msgs_GoalInfo;

typedef struct {
    action_msgs_GoalInfo goal_info;
    int8_t status;
} action_msgs_GoalStatus;

bool action_msgs_GoalStatus_encode(const action_msgs_GoalStatus *msg, uint8_t *buf, size_t cap,
                                    size_t *out_len);
bool action_msgs_GoalStatus_decode(const uint8_t *buf, size_t len, action_msgs_GoalStatus *out);

typedef struct {
    action_msgs_GoalStatus status_list[XRCE_MSGS_GOAL_STATUS_MAX];
    uint32_t status_list_count;
} action_msgs_GoalStatusArray;

bool action_msgs_GoalStatusArray_encode(const action_msgs_GoalStatusArray *msg, uint8_t *buf, size_t cap,
                                         size_t *out_len);
bool action_msgs_GoalStatusArray_decode(const uint8_t *buf, size_t len, action_msgs_GoalStatusArray *out);

typedef struct {
    action_msgs_GoalInfo goal_info;
} action_msgs_CancelGoal_Request;

bool action_msgs_CancelGoal_Request_encode(const action_msgs_CancelGoal_Request *msg, uint8_t *buf,
                                            size_t cap, size_t *out_len);
bool action_msgs_CancelGoal_Request_decode(const uint8_t *buf, size_t len,
                                            action_msgs_CancelGoal_Request *out);

typedef struct {
    int8_t return_code;
    action_msgs_GoalInfo goals_canceling[XRCE_MSGS_GOAL_STATUS_MAX];
    uint32_t goals_canceling_count;
} action_msgs_CancelGoal_Response;

bool action_msgs_CancelGoal_Response_encode(const action_msgs_CancelGoal_Response *msg, uint8_t *buf,
                                             size_t cap, size_t *out_len);
bool action_msgs_CancelGoal_Response_decode(const uint8_t *buf, size_t len,
                                             action_msgs_CancelGoal_Response *out);

/* Phase 8: diagnostic_msgs/msg/DiagnosticArray -- a real, already-installed
 * standard interface (confirmed via `ros2 interface show
 * diagnostic_msgs/msg/DiagnosticArray`), chosen so `/rtos/diagnostics`
 * is idiomatic and, in principle, `rqt_robot_monitor`-compatible, per the
 * phase brief's own suggestion, rather than a custom message type. Field
 * layout: Header{Time stamp; string frame_id} header;
 * DiagnosticStatus{byte level; string name/message/hardware_id;
 * KeyValue[] values} status[]. */
#define XRCE_MSGS_KV_MAX 8
#define XRCE_MSGS_DIAG_STATUS_MAX 16
#define XRCE_MSGS_DIAG_STR_MAX 64

#define DIAGNOSTIC_STATUS_OK 0
#define DIAGNOSTIC_STATUS_WARN 1
#define DIAGNOSTIC_STATUS_ERROR 2
#define DIAGNOSTIC_STATUS_STALE 3

typedef struct {
    char key[XRCE_MSGS_DIAG_STR_MAX];
    char value[XRCE_MSGS_DIAG_STR_MAX];
} diagnostic_msgs_KeyValue;

typedef struct {
    uint8_t level;
    char name[XRCE_MSGS_DIAG_STR_MAX];
    char message[XRCE_MSGS_DIAG_STR_MAX];
    char hardware_id[XRCE_MSGS_DIAG_STR_MAX];
    diagnostic_msgs_KeyValue values[XRCE_MSGS_KV_MAX];
    uint32_t values_count;
} diagnostic_msgs_DiagnosticStatus;

typedef struct {
    std_msgs_Header header;
    diagnostic_msgs_DiagnosticStatus status[XRCE_MSGS_DIAG_STATUS_MAX];
    uint32_t status_count;
} diagnostic_msgs_DiagnosticArray;

bool diagnostic_msgs_DiagnosticArray_encode(const diagnostic_msgs_DiagnosticArray *msg, uint8_t *buf,
                                             size_t cap, size_t *out_len);
bool diagnostic_msgs_DiagnosticArray_decode(const uint8_t *buf, size_t len,
                                             diagnostic_msgs_DiagnosticArray *out);

#endif
