#include "xrce/msgs.h"

bool std_msgs_Int32_encode(const std_msgs_Int32 *msg, uint8_t *buf, size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_i32(&w, msg->data)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool std_msgs_Int32_decode(const uint8_t *buf, size_t len, std_msgs_Int32 *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_i32(&r, &out->data);
}

bool std_msgs_String_encode(const std_msgs_String *msg, uint8_t *buf, size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_string(&w, msg->data)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool std_msgs_String_decode(const uint8_t *buf, size_t len, std_msgs_String *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) &&
           xrce_cdr_read_string(&r, out->data, sizeof(out->data));
}

static bool write_vec3(xrce_cdr_writer_t *w, const geometry_msgs_Vector3 *v) {
    return xrce_cdr_write_f64(w, v->x) && xrce_cdr_write_f64(w, v->y) &&
           xrce_cdr_write_f64(w, v->z);
}

static bool read_vec3(xrce_cdr_reader_t *r, geometry_msgs_Vector3 *v) {
    return xrce_cdr_read_f64(r, &v->x) && xrce_cdr_read_f64(r, &v->y) &&
           xrce_cdr_read_f64(r, &v->z);
}

static bool write_cov(xrce_cdr_writer_t *w, const double cov[9]) {
    for (int i = 0; i < 9; i++) {
        if (!xrce_cdr_write_f64(w, cov[i])) {
            return false;
        }
    }
    return true;
}

static bool read_cov(xrce_cdr_reader_t *r, double cov[9]) {
    for (int i = 0; i < 9; i++) {
        if (!xrce_cdr_read_f64(r, &cov[i])) {
            return false;
        }
    }
    return true;
}

bool sensor_msgs_Imu_encode(const sensor_msgs_Imu *msg, uint8_t *buf, size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    if (!xrce_cdr_write_header(&w) ||
        !xrce_cdr_write_i32(&w, msg->header.stamp.sec) ||
        !xrce_cdr_write_u32(&w, msg->header.stamp.nanosec) ||
        !xrce_cdr_write_string(&w, msg->header.frame_id) ||
        !xrce_cdr_write_f64(&w, msg->orientation.x) ||
        !xrce_cdr_write_f64(&w, msg->orientation.y) ||
        !xrce_cdr_write_f64(&w, msg->orientation.z) ||
        !xrce_cdr_write_f64(&w, msg->orientation.w) ||
        !write_cov(&w, msg->orientation_covariance) ||
        !write_vec3(&w, &msg->angular_velocity) ||
        !write_cov(&w, msg->angular_velocity_covariance) ||
        !write_vec3(&w, &msg->linear_acceleration) ||
        !write_cov(&w, msg->linear_acceleration_covariance)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool sensor_msgs_Imu_decode(const uint8_t *buf, size_t len, sensor_msgs_Imu *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);

    return xrce_cdr_read_header(&r) &&
           xrce_cdr_read_i32(&r, &out->header.stamp.sec) &&
           xrce_cdr_read_u32(&r, &out->header.stamp.nanosec) &&
           xrce_cdr_read_string(&r, out->header.frame_id, sizeof(out->header.frame_id)) &&
           xrce_cdr_read_f64(&r, &out->orientation.x) &&
           xrce_cdr_read_f64(&r, &out->orientation.y) &&
           xrce_cdr_read_f64(&r, &out->orientation.z) &&
           xrce_cdr_read_f64(&r, &out->orientation.w) &&
           read_cov(&r, out->orientation_covariance) &&
           read_vec3(&r, &out->angular_velocity) &&
           read_cov(&r, out->angular_velocity_covariance) &&
           read_vec3(&r, &out->linear_acceleration) &&
           read_cov(&r, out->linear_acceleration_covariance);
}

bool std_srvs_Trigger_Request_encode(const std_srvs_Trigger_Request *msg, uint8_t *buf, size_t cap,
                                      size_t *out_len) {
    (void)msg;
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool std_srvs_Trigger_Request_decode(const uint8_t *buf, size_t len, std_srvs_Trigger_Request *out) {
    (void)out;
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r);
}

bool std_srvs_Trigger_Response_encode(const std_srvs_Trigger_Response *msg, uint8_t *buf, size_t cap,
                                       size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_bool(&w, msg->success) ||
        !xrce_cdr_write_string(&w, msg->message)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool std_srvs_Trigger_Response_decode(const uint8_t *buf, size_t len, std_srvs_Trigger_Response *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_bool(&r, &out->success) &&
           xrce_cdr_read_string(&r, out->message, sizeof(out->message));
}

static bool write_fibonacci_result(xrce_cdr_writer_t *w, const example_interfaces_Fibonacci_Result *v) {
    return xrce_cdr_write_seq_i32(w, v->sequence, v->sequence_count);
}

static bool read_fibonacci_result(xrce_cdr_reader_t *r, example_interfaces_Fibonacci_Result *v) {
    return xrce_cdr_read_seq_i32(r, v->sequence, XRCE_MSGS_SEQ_I32_MAX, &v->sequence_count);
}

bool example_interfaces_Fibonacci_SendGoal_Request_encode(
    const example_interfaces_Fibonacci_SendGoal_Request *msg, uint8_t *buf, size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_bytes(&w, msg->goal_id, XRCE_MSGS_UUID_SIZE) ||
        !xrce_cdr_write_i32(&w, msg->order)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool example_interfaces_Fibonacci_SendGoal_Request_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_SendGoal_Request *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_bytes(&r, out->goal_id, XRCE_MSGS_UUID_SIZE) &&
           xrce_cdr_read_i32(&r, &out->order);
}

bool example_interfaces_Fibonacci_SendGoal_Response_encode(
    const example_interfaces_Fibonacci_SendGoal_Response *msg, uint8_t *buf, size_t cap,
    size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_bool(&w, msg->accepted) ||
        !xrce_cdr_write_i32(&w, msg->stamp.sec) || !xrce_cdr_write_u32(&w, msg->stamp.nanosec)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool example_interfaces_Fibonacci_SendGoal_Response_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_SendGoal_Response *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_bool(&r, &out->accepted) &&
           xrce_cdr_read_i32(&r, &out->stamp.sec) && xrce_cdr_read_u32(&r, &out->stamp.nanosec);
}

bool example_interfaces_Fibonacci_GetResult_Request_encode(
    const example_interfaces_Fibonacci_GetResult_Request *msg, uint8_t *buf, size_t cap,
    size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_bytes(&w, msg->goal_id, XRCE_MSGS_UUID_SIZE)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool example_interfaces_Fibonacci_GetResult_Request_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_GetResult_Request *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_bytes(&r, out->goal_id, XRCE_MSGS_UUID_SIZE);
}

bool example_interfaces_Fibonacci_GetResult_Response_encode(
    const example_interfaces_Fibonacci_GetResult_Response *msg, uint8_t *buf, size_t cap,
    size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_u8(&w, (uint8_t)msg->status) ||
        !write_fibonacci_result(&w, &msg->result)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool example_interfaces_Fibonacci_GetResult_Response_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_GetResult_Response *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    uint8_t status;
    if (!xrce_cdr_read_header(&r) || !xrce_cdr_read_u8(&r, &status) ||
        !read_fibonacci_result(&r, &out->result)) {
        return false;
    }
    out->status = (int8_t)status;
    return true;
}

bool example_interfaces_Fibonacci_FeedbackMessage_encode(
    const example_interfaces_Fibonacci_FeedbackMessage *msg, uint8_t *buf, size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_bytes(&w, msg->goal_id, XRCE_MSGS_UUID_SIZE) ||
        !write_fibonacci_result(&w, &msg->feedback)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool example_interfaces_Fibonacci_FeedbackMessage_decode(
    const uint8_t *buf, size_t len, example_interfaces_Fibonacci_FeedbackMessage *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && xrce_cdr_read_bytes(&r, out->goal_id, XRCE_MSGS_UUID_SIZE) &&
           read_fibonacci_result(&r, &out->feedback);
}

static bool write_goal_info(xrce_cdr_writer_t *w, const action_msgs_GoalInfo *v) {
    return xrce_cdr_write_bytes(w, v->goal_id, XRCE_MSGS_UUID_SIZE) &&
           xrce_cdr_write_i32(w, v->stamp.sec) && xrce_cdr_write_u32(w, v->stamp.nanosec);
}

static bool read_goal_info(xrce_cdr_reader_t *r, action_msgs_GoalInfo *v) {
    return xrce_cdr_read_bytes(r, v->goal_id, XRCE_MSGS_UUID_SIZE) &&
           xrce_cdr_read_i32(r, &v->stamp.sec) && xrce_cdr_read_u32(r, &v->stamp.nanosec);
}

static bool write_goal_status(xrce_cdr_writer_t *w, const action_msgs_GoalStatus *v) {
    return write_goal_info(w, &v->goal_info) && xrce_cdr_write_u8(w, (uint8_t)v->status);
}

static bool read_goal_status(xrce_cdr_reader_t *r, action_msgs_GoalStatus *v) {
    uint8_t status;
    if (!read_goal_info(r, &v->goal_info) || !xrce_cdr_read_u8(r, &status)) {
        return false;
    }
    v->status = (int8_t)status;
    return true;
}

bool action_msgs_GoalStatus_encode(const action_msgs_GoalStatus *msg, uint8_t *buf, size_t cap,
                                    size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !write_goal_status(&w, msg)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool action_msgs_GoalStatus_decode(const uint8_t *buf, size_t len, action_msgs_GoalStatus *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && read_goal_status(&r, out);
}

bool action_msgs_GoalStatusArray_encode(const action_msgs_GoalStatusArray *msg, uint8_t *buf, size_t cap,
                                         size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_u32(&w, msg->status_list_count)) {
        return false;
    }
    for (uint32_t i = 0; i < msg->status_list_count; i++) {
        if (!write_goal_status(&w, &msg->status_list[i])) {
            return false;
        }
    }
    *out_len = w.pos;
    return true;
}

bool action_msgs_GoalStatusArray_decode(const uint8_t *buf, size_t len, action_msgs_GoalStatusArray *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    uint32_t count;
    if (!xrce_cdr_read_header(&r) || !xrce_cdr_read_u32(&r, &count) || count > XRCE_MSGS_GOAL_STATUS_MAX) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!read_goal_status(&r, &out->status_list[i])) {
            return false;
        }
    }
    out->status_list_count = count;
    return true;
}

bool action_msgs_CancelGoal_Request_encode(const action_msgs_CancelGoal_Request *msg, uint8_t *buf,
                                            size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !write_goal_info(&w, &msg->goal_info)) {
        return false;
    }
    *out_len = w.pos;
    return true;
}

bool action_msgs_CancelGoal_Request_decode(const uint8_t *buf, size_t len,
                                            action_msgs_CancelGoal_Request *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    return xrce_cdr_read_header(&r) && read_goal_info(&r, &out->goal_info);
}

bool action_msgs_CancelGoal_Response_encode(const action_msgs_CancelGoal_Response *msg, uint8_t *buf,
                                             size_t cap, size_t *out_len) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);
    if (!xrce_cdr_write_header(&w) || !xrce_cdr_write_u8(&w, (uint8_t)msg->return_code) ||
        !xrce_cdr_write_u32(&w, msg->goals_canceling_count)) {
        return false;
    }
    for (uint32_t i = 0; i < msg->goals_canceling_count; i++) {
        if (!write_goal_info(&w, &msg->goals_canceling[i])) {
            return false;
        }
    }
    *out_len = w.pos;
    return true;
}

bool action_msgs_CancelGoal_Response_decode(const uint8_t *buf, size_t len,
                                             action_msgs_CancelGoal_Response *out) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    uint8_t return_code;
    uint32_t count;
    if (!xrce_cdr_read_header(&r) || !xrce_cdr_read_u8(&r, &return_code) ||
        !xrce_cdr_read_u32(&r, &count) || count > XRCE_MSGS_GOAL_STATUS_MAX) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!read_goal_info(&r, &out->goals_canceling[i])) {
            return false;
        }
    }
    out->return_code = (int8_t)return_code;
    out->goals_canceling_count = count;
    return true;
}
