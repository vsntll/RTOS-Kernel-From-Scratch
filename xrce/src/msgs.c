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
