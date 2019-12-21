#define MAGICFULL 0xfefe
#define MAGIC 0xfe
#define N_MAGIC 2

#define MSG_ID_RPM 0
#define MSG_ID_ENABLE 1
#define MSG_ID_CMD 2
#define MSG_ID_IMU 3
#define MSG_ID_GAINS 4

#define META_SIZE 5

#define MSG_START(x) \
  struct __attribute__ ((__packed__ )) x##_msg { \
    uint16_t magic; \
    uint8_t length; \
    uint8_t sequence; \
    uint8_t msg_id;
#define MSG_END \
    uint8_t csum; \
  };

MSG_START(rpm)
  uint64_t timestamp;
  uint16_t rpm[4];
MSG_END

MSG_START(enable)
  uint8_t enable;
MSG_END

MSG_START(cmd)
  uint64_t timestamp;
  float thrust;
  float q[4];
  float yaw;
MSG_END

MSG_START(imu)
  uint64_t timestamp;
  float accel[3];
  float gyro[3];
  float accel_filt[3];
  float gyro_filt[3];
  float roll;
  float pitch;
MSG_END

MSG_START(gains)
  float kR[3];
  float kOm[3];
MSG_END
