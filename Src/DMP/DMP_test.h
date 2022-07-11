#ifdef  _DMP_TEST_H_
#define _DMP_TEST_H_

extern float pitch;
extern float yaw;
extern float roll;
float roll;
float yaw;
float pitch;
unsigned char run_self_test(void);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);
void mget_ms(unsigned long *time);
unsigned char mpu_dmp_init(void);
unsigned char mpu_dmp_get_data(float *pitch,float *roll,float *yaw);
int DMP_test(void);

#endif

