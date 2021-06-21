#define MAGIC 0xfe
#define N_MAGIC 1

#define META_SIZE 2

#define FILECHUNK_SIZE 64
#define MAX_FILENAME_LENGTH 32

#define MSG_START(x, y) \
  static constexpr uint8_t MSG_ID_##x = y; \
  struct __attribute__ ((__packed__ )) x##_msg { \
    uint8_t magic; \
    uint8_t msg_id;
#define MSG_END \
    uint16_t csum; \
  };

MSG_START(rpm, 0)
  uint64_t timestamp;
  uint16_t rpm[4];
MSG_END

MSG_START(enable, 1)
  uint8_t enable;
MSG_END

MSG_START(cmd, 2)
  uint64_t timestamp;
  float thrust;
  float q[4];
  float angvel[3];
  float angacc[3];
  float yaw;
MSG_END

MSG_START(imu, 3)
  uint64_t timestamp;
  float accel[3];
  float gyro[3];
  float accel_filt[3];
  float gyro_filt[3];
  float roll;
  float pitch;
MSG_END

MSG_START(gains, 4)
  float kR[3];
  float kOm[3];
MSG_END

MSG_START(flcmd, 5)
  uint64_t timestamp;
  float snap_ff[3];
  float desired_yaw;
  float desired_yawacc;
  float yaw;
MSG_END

MSG_START(flgains, 6)
  float k3[3];
  float k4[3];
  float yaw_kp;
  float yaw_kd;
  float delay_const;
MSG_END

MSG_START(flstate, 7)
  uint64_t timestamp;
  float u;
  float udot;
MSG_END

MSG_START(tvcmd, 8)
  uint64_t timestamp;
  float accel[3];
  float angacc[3];
  float desired_yaw;
  float yaw;
MSG_END

MSG_START(ack, 9)
  uint8_t id;
MSG_END

MSG_START(file_request, 10)
  char filename[MAX_FILENAME_LENGTH];
MSG_END

MSG_START(file_chunk, 11)
  uint8_t id;
  uint8_t length;
  uint8_t data[FILECHUNK_SIZE];
MSG_END

MSG_START(accel, 12)
  float accel[3];
MSG_END

MSG_START(gps, 13)
  uint64_t timestamp;
  uint64_t time_utc;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int32_t alt_ellipsoid;
  float s_variance_m_s;
  float c_variance_rad;
  float eph;
  float epv;
  float hdop;
  float vdop;
  int32_t noise_per_ms;
  int32_t jamming_indicator;
  float vel_m_s;
  float vel_ned[3];
  float cog_rad;
  uint8_t fix_type;
  bool vel_ned_valid;
  uint8_t satellites_used;
MSG_END
