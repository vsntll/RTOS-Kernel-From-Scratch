/* Phase 2 milestone (ROS2 layer): round-trips the three message types
 * through their real field layout (std_msgs/Int32, std_msgs/String,
 * sensor_msgs/Imu), the last one specifically to prove nested structs and
 * mixed-alignment fields (int32/uint32 header fields next to float64
 * vectors) survive encode->decode intact. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/msgs.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void case_int32_round_trip(void) {
    std_msgs_Int32 in = {.data = -12345};
    uint8_t buf[16];
    size_t len;
    assert(std_msgs_Int32_encode(&in, buf, sizeof(buf), &len));

    std_msgs_Int32 out;
    assert(std_msgs_Int32_decode(buf, len, &out));
    assert(out.data == in.data);
}

static void case_string_round_trip(void) {
    std_msgs_String in;
    strcpy(in.data, "hello from the RTOS");
    uint8_t buf[64];
    size_t len;
    assert(std_msgs_String_encode(&in, buf, sizeof(buf), &len));

    std_msgs_String out;
    assert(std_msgs_String_decode(buf, len, &out));
    assert(strcmp(out.data, in.data) == 0);
}

static void case_imu_round_trip(void) {
    sensor_msgs_Imu in = {0};
    in.header.stamp.sec = 1700000000;
    in.header.stamp.nanosec = 500000000u;
    strcpy(in.header.frame_id, "imu_link");
    in.orientation = (geometry_msgs_Quaternion){.x = 0.1, .y = 0.2, .z = 0.3, .w = 0.9};
    for (int i = 0; i < 9; i++) {
        in.orientation_covariance[i] = (double)i * 0.01;
        in.angular_velocity_covariance[i] = (double)i * 0.02;
        in.linear_acceleration_covariance[i] = (double)i * 0.03;
    }
    in.angular_velocity = (geometry_msgs_Vector3){.x = 1.0, .y = -1.0, .z = 0.5};
    in.linear_acceleration = (geometry_msgs_Vector3){.x = 0.0, .y = 0.0, .z = 9.81};

    uint8_t buf[512]; /* full Imu layout: header + stamp + frame_id + 3
                        * quaternion/vector fields + three 9-double
                        * covariance arrays comes to 328 bytes */
    size_t len;
    assert(sensor_msgs_Imu_encode(&in, buf, sizeof(buf), &len));

    sensor_msgs_Imu out;
    assert(sensor_msgs_Imu_decode(buf, len, &out));

    assert(out.header.stamp.sec == in.header.stamp.sec);
    assert(out.header.stamp.nanosec == in.header.stamp.nanosec);
    assert(strcmp(out.header.frame_id, in.header.frame_id) == 0);
    assert(out.orientation.x == in.orientation.x && out.orientation.w == in.orientation.w);
    assert(memcmp(out.orientation_covariance, in.orientation_covariance,
                   sizeof(in.orientation_covariance)) == 0);
    assert(out.angular_velocity.z == in.angular_velocity.z);
    assert(memcmp(out.angular_velocity_covariance, in.angular_velocity_covariance,
                   sizeof(in.angular_velocity_covariance)) == 0);
    assert(out.linear_acceleration.z == in.linear_acceleration.z);
    assert(memcmp(out.linear_acceleration_covariance, in.linear_acceleration_covariance,
                   sizeof(in.linear_acceleration_covariance)) == 0);
}

static void case_imu_encode_fails_on_small_buffer(void) {
    sensor_msgs_Imu in = {0};
    strcpy(in.header.frame_id, "x");
    uint8_t buf[8]; /* nowhere near enough for the full Imu layout */
    size_t len;
    assert(!sensor_msgs_Imu_encode(&in, buf, sizeof(buf), &len));
}

/* Phase 14: geometry_msgs/Twist -- just two Vector3s, reusing the same
 * write_vec3()/read_vec3() helpers Imu's angular_velocity/
 * linear_acceleration fields already exercise, so this is a lighter
 * round-trip check than Imu's rather than a repeat of the same ground. */
static void case_twist_round_trip(void) {
    geometry_msgs_Twist in = {
        .linear = {.x = 0.5, .y = 0.0, .z = 0.0},
        .angular = {.x = 0.0, .y = 0.0, .z = 0.3},
    };
    uint8_t buf[64];
    size_t len;
    assert(geometry_msgs_Twist_encode(&in, buf, sizeof(buf), &len));
    /* 4-byte header + 6 float64 fields, no padding between them: CDR
     * alignment resets relative to right after the header (xrce/include/
     * xrce/cdr.h's align_base), not absolute buffer position -- confirmed
     * against a real `rclpy.serialization.serialize_message()` capture of
     * an actual geometry_msgs/Twist, see xrce/docs/design.md's Phase 14
     * section for the exact bytes compared. */
    assert(len == 4 + 6 * 8);

    geometry_msgs_Twist out;
    assert(geometry_msgs_Twist_decode(buf, len, &out));
    assert(out.linear.x == in.linear.x && out.linear.y == in.linear.y && out.linear.z == in.linear.z);
    assert(out.angular.x == in.angular.x && out.angular.y == in.angular.y &&
           out.angular.z == in.angular.z);
}

static void case_twist_encode_fails_on_small_buffer(void) {
    geometry_msgs_Twist in = {0};
    uint8_t buf[8];
    size_t len;
    assert(!geometry_msgs_Twist_encode(&in, buf, sizeof(buf), &len));
}

static void case_trigger_request_round_trip(void) {
    std_srvs_Trigger_Request in = {0};
    uint8_t buf[16];
    size_t len;
    assert(std_srvs_Trigger_Request_encode(&in, buf, sizeof(buf), &len));
    assert(len == 4); /* header only -- Trigger's request has no fields */

    std_srvs_Trigger_Request out;
    assert(std_srvs_Trigger_Request_decode(buf, len, &out));
}

static void case_trigger_response_round_trip(void) {
    std_srvs_Trigger_Response in = {.success = true};
    strcpy(in.message, "self-test #1 ok");
    uint8_t buf[64];
    size_t len;
    assert(std_srvs_Trigger_Response_encode(&in, buf, sizeof(buf), &len));

    std_srvs_Trigger_Response out;
    assert(std_srvs_Trigger_Response_decode(buf, len, &out));
    assert(out.success == in.success);
    assert(strcmp(out.message, in.message) == 0);
}

static void fill_uuid(uint8_t goal_id[XRCE_MSGS_UUID_SIZE], uint8_t seed) {
    for (int i = 0; i < XRCE_MSGS_UUID_SIZE; i++) {
        goal_id[i] = (uint8_t)(seed + i);
    }
}

static void case_send_goal_round_trip(void) {
    example_interfaces_Fibonacci_SendGoal_Request in = {.order = 7};
    fill_uuid(in.goal_id, 0x10);
    uint8_t buf[64];
    size_t len;
    assert(example_interfaces_Fibonacci_SendGoal_Request_encode(&in, buf, sizeof(buf), &len));

    example_interfaces_Fibonacci_SendGoal_Request out;
    assert(example_interfaces_Fibonacci_SendGoal_Request_decode(buf, len, &out));
    assert(out.order == in.order);
    assert(memcmp(out.goal_id, in.goal_id, XRCE_MSGS_UUID_SIZE) == 0);

    example_interfaces_Fibonacci_SendGoal_Response resp_in = {.accepted = true,
                                                                .stamp = {.sec = 5, .nanosec = 6}};
    assert(example_interfaces_Fibonacci_SendGoal_Response_encode(&resp_in, buf, sizeof(buf), &len));
    example_interfaces_Fibonacci_SendGoal_Response resp_out;
    assert(example_interfaces_Fibonacci_SendGoal_Response_decode(buf, len, &resp_out));
    assert(resp_out.accepted == resp_in.accepted);
    assert(resp_out.stamp.sec == 5 && resp_out.stamp.nanosec == 6);
}

static void case_get_result_round_trip(void) {
    example_interfaces_Fibonacci_GetResult_Request in;
    fill_uuid(in.goal_id, 0x20);
    uint8_t buf[64];
    size_t len;
    assert(example_interfaces_Fibonacci_GetResult_Request_encode(&in, buf, sizeof(buf), &len));
    example_interfaces_Fibonacci_GetResult_Request out;
    assert(example_interfaces_Fibonacci_GetResult_Request_decode(buf, len, &out));
    assert(memcmp(out.goal_id, in.goal_id, XRCE_MSGS_UUID_SIZE) == 0);

    example_interfaces_Fibonacci_GetResult_Response resp_in = {.status = 4};
    resp_in.result.sequence_count = 3;
    resp_in.result.sequence[0] = 0;
    resp_in.result.sequence[1] = 1;
    resp_in.result.sequence[2] = 1;
    assert(example_interfaces_Fibonacci_GetResult_Response_encode(&resp_in, buf, sizeof(buf), &len));
    example_interfaces_Fibonacci_GetResult_Response resp_out;
    assert(example_interfaces_Fibonacci_GetResult_Response_decode(buf, len, &resp_out));
    assert(resp_out.status == 4);
    assert(resp_out.result.sequence_count == 3);
    assert(resp_out.result.sequence[2] == 1);
}

static void case_feedback_message_round_trip(void) {
    example_interfaces_Fibonacci_FeedbackMessage in = {0};
    fill_uuid(in.goal_id, 0x30);
    in.feedback.sequence_count = 2;
    in.feedback.sequence[0] = 0;
    in.feedback.sequence[1] = 1;

    uint8_t buf[64];
    size_t len;
    assert(example_interfaces_Fibonacci_FeedbackMessage_encode(&in, buf, sizeof(buf), &len));
    example_interfaces_Fibonacci_FeedbackMessage out;
    assert(example_interfaces_Fibonacci_FeedbackMessage_decode(buf, len, &out));
    assert(memcmp(out.goal_id, in.goal_id, XRCE_MSGS_UUID_SIZE) == 0);
    assert(out.feedback.sequence_count == 2 && out.feedback.sequence[1] == 1);
}

static void case_goal_status_array_round_trip(void) {
    action_msgs_GoalStatusArray in = {0};
    in.status_list_count = 2;
    fill_uuid(in.status_list[0].goal_info.goal_id, 0x40);
    in.status_list[0].goal_info.stamp = (builtin_interfaces_Time){.sec = 1, .nanosec = 2};
    in.status_list[0].status = 2; /* EXECUTING */
    fill_uuid(in.status_list[1].goal_info.goal_id, 0x50);
    in.status_list[1].status = 4; /* SUCCEEDED */

    uint8_t buf[128];
    size_t len;
    assert(action_msgs_GoalStatusArray_encode(&in, buf, sizeof(buf), &len));
    action_msgs_GoalStatusArray out;
    assert(action_msgs_GoalStatusArray_decode(buf, len, &out));
    assert(out.status_list_count == 2);
    assert(out.status_list[0].status == 2);
    assert(out.status_list[1].status == 4);
    assert(memcmp(out.status_list[1].goal_info.goal_id, in.status_list[1].goal_info.goal_id,
                   XRCE_MSGS_UUID_SIZE) == 0);
}

static void case_cancel_goal_round_trip(void) {
    action_msgs_CancelGoal_Request in = {0};
    fill_uuid(in.goal_info.goal_id, 0x60);
    uint8_t buf[64];
    size_t len;
    assert(action_msgs_CancelGoal_Request_encode(&in, buf, sizeof(buf), &len));
    action_msgs_CancelGoal_Request out;
    assert(action_msgs_CancelGoal_Request_decode(buf, len, &out));
    assert(memcmp(out.goal_info.goal_id, in.goal_info.goal_id, XRCE_MSGS_UUID_SIZE) == 0);

    action_msgs_CancelGoal_Response resp_in = {.return_code = 0, .goals_canceling_count = 1};
    fill_uuid(resp_in.goals_canceling[0].goal_id, 0x60);
    assert(action_msgs_CancelGoal_Response_encode(&resp_in, buf, sizeof(buf), &len));
    action_msgs_CancelGoal_Response resp_out;
    assert(action_msgs_CancelGoal_Response_decode(buf, len, &resp_out));
    assert(resp_out.goals_canceling_count == 1);
    assert(memcmp(resp_out.goals_canceling[0].goal_id, resp_in.goals_canceling[0].goal_id,
                   XRCE_MSGS_UUID_SIZE) == 0);
}

static void case_diagnostic_array_round_trip(void) {
    diagnostic_msgs_DiagnosticArray in = {0};
    in.header.stamp.sec = 10;
    in.header.stamp.nanosec = 20;
    strcpy(in.header.frame_id, "rtos");
    in.status_count = 2;

    in.status[0].level = DIAGNOSTIC_STATUS_OK;
    strcpy(in.status[0].name, "task/high");
    strcpy(in.status[0].message, "RUNNING");
    strcpy(in.status[0].hardware_id, "rtos");
    in.status[0].values_count = 2;
    strcpy(in.status[0].values[0].key, "priority");
    strcpy(in.status[0].values[0].value, "50");
    strcpy(in.status[0].values[1].key, "stack_hwm");
    strcpy(in.status[0].values[1].value, "1024");

    in.status[1].level = DIAGNOSTIC_STATUS_WARN;
    strcpy(in.status[1].name, "task/low");
    strcpy(in.status[1].message, "BLOCKED");
    strcpy(in.status[1].hardware_id, "rtos");
    in.status[1].values_count = 0;

    uint8_t buf[1024];
    size_t len;
    assert(diagnostic_msgs_DiagnosticArray_encode(&in, buf, sizeof(buf), &len));

    diagnostic_msgs_DiagnosticArray out;
    assert(diagnostic_msgs_DiagnosticArray_decode(buf, len, &out));
    assert(out.header.stamp.sec == 10 && out.header.stamp.nanosec == 20);
    assert(strcmp(out.header.frame_id, "rtos") == 0);
    assert(out.status_count == 2);
    assert(out.status[0].level == DIAGNOSTIC_STATUS_OK);
    assert(strcmp(out.status[0].name, "task/high") == 0);
    assert(out.status[0].values_count == 2);
    assert(strcmp(out.status[0].values[1].key, "stack_hwm") == 0);
    assert(strcmp(out.status[0].values[1].value, "1024") == 0);
    assert(out.status[1].level == DIAGNOSTIC_STATUS_WARN);
    assert(out.status[1].values_count == 0);
}

int main(void) {
    run_case("std_msgs/Int32 round trip", case_int32_round_trip);
    run_case("std_msgs/String round trip", case_string_round_trip);
    run_case("sensor_msgs/Imu round trip (nested + mixed alignment)", case_imu_round_trip);
    run_case("Imu encode fails cleanly on undersized buffer", case_imu_encode_fails_on_small_buffer);
    run_case("geometry_msgs/Twist round trip", case_twist_round_trip);
    run_case("Twist encode fails cleanly on undersized buffer", case_twist_encode_fails_on_small_buffer);
    run_case("std_srvs/Trigger request round trip (no fields)", case_trigger_request_round_trip);
    run_case("std_srvs/Trigger response round trip", case_trigger_response_round_trip);
    run_case("Fibonacci SendGoal request/response round trip", case_send_goal_round_trip);
    run_case("Fibonacci GetResult request/response round trip", case_get_result_round_trip);
    run_case("Fibonacci FeedbackMessage round trip", case_feedback_message_round_trip);
    run_case("action_msgs/GoalStatusArray round trip", case_goal_status_array_round_trip);
    run_case("action_msgs/CancelGoal request/response round trip", case_cancel_goal_round_trip);
    run_case("diagnostic_msgs/DiagnosticArray round trip", case_diagnostic_array_round_trip);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
