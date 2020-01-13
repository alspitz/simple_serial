#define MAGICFULL 0xfefe
#define MAGIC 0xfe
#define N_MAGIC 2

#define META_SIZE 5

#define MSG_START(x, y) \
  static constexpr uint8_t MSG_ID_##x = y; \
  struct __attribute__ ((__packed__ )) x##_msg { \
    uint16_t magic; \
    uint8_t length; \
    uint8_t sequence; \
    uint8_t msg_id;
#define MSG_END \
    uint8_t csum; \
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
  float v1_ilc;
  float angacc_ilc[3];
  float desired_yaw;
  float yaw;
MSG_END

MSG_START(flgains, 6)
  float k3[3];
  float k4[3];
  float yaw_kp;
  float yaw_kd;
MSG_END

MSG_START(flstate, 7)
  uint64_t timestamp;
  float u;
  float udot;
MSG_END
