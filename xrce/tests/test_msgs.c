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

int main(void) {
    run_case("std_msgs/Int32 round trip", case_int32_round_trip);
    run_case("std_msgs/String round trip", case_string_round_trip);
    run_case("sensor_msgs/Imu round trip (nested + mixed alignment)", case_imu_round_trip);
    run_case("Imu encode fails cleanly on undersized buffer", case_imu_encode_fails_on_small_buffer);
    run_case("std_srvs/Trigger request round trip (no fields)", case_trigger_request_round_trip);
    run_case("std_srvs/Trigger response round trip", case_trigger_response_round_trip);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
