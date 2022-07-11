//#include "pbdata.h"

#include "i2c.h"

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)

#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)

#define	GYRO_CONFIG		0x1B	
//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
/*
0bxx<<3:
00 ~ +-250��
01 ~ +-500��
10 ~ +-1000��
11 ~ +-2000��
*/


#define	ACCEL_CONFIG	0x1C	
//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
/*
0bxx<<3:
00 ~ +-2g
01 ~ +-4g
10 ~ +-18g
11 ~ +-16g
*/

#define	ACCEL_XOUT_H	0x3B//���ٶȼƲ���ֵ�Ĵ�����ַ
#define	ACCEL_XOUT_L	0x3C 

#define	ACCEL_YOUT_H	0x3D //2 3
#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
/********************/
#define	TEMP_OUT_H		0x41//�¶Ȳ���ֵ�Ĵ�����ַ
#define	TEMP_OUT_L		0x42
/*********************/
#define	GYRO_XOUT_H		0x43// 8 9 �����ǲ���ֵ�Ĵ�����ַ
#define	GYRO_XOUT_L		0x44
	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define MPU6050_ADDR  	0xD0

#define MPU6050_WHO_AM_I         0x75

unsigned char IIC_buf[14];

signed short ax;
signed short ay;
signed short az;

signed short gx;
signed short gy;
signed short gz;

signed short ax_cl = 0;
signed short ay_cl = 0;
signed short az_cl = 0;

signed short gx_cl = 0;
signed short gy_cl = 0;
signed short gz_cl = 0;


void IIC_Write(unsigned char dev_addr, unsigned char mem_addr, 
								unsigned char data)
{
	HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr,
						I2C_MEMADD_SIZE_8BIT, &data, 1, 2);
}

void IIC_Read(unsigned char dev_addr, unsigned char mem_addr, 
	unsigned char *buf, unsigned char len)
{
	HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, 
					I2C_MEMADD_SIZE_8BIT, buf, len, 2);
}

/*д���ʼ��MPU6050*/
void MPU6050_Init(void)
{
	unsigned char temp = 0;
reset_MPU6050:
	IIC_Write(MPU6050_ADDR,PWR_MGMT_1,0x80);	//��λMPU6050
	HAL_Delay(50);
	IIC_Write(MPU6050_ADDR,PWR_MGMT_1,0x01);	//����MPU6050��ʹ��x��PLLΪʱ��
	IIC_Read(MPU6050_ADDR, MPU6050_WHO_AM_I, &temp, 1);	//��ID
	temp &= 0x7e;
	printf("%d\r\n",temp);
	if(temp != 0x68)
		goto reset_MPU6050;
		
	
	IIC_Write(MPU6050_ADDR,SMPLRT_DIV,0x00);	//�����ʷ�Ƶ������Ƶ
	IIC_Write(MPU6050_ADDR,CONFIG,0x00);	//��ͨ�˲���������256~260Hz
	
	IIC_Write(MPU6050_ADDR,GYRO_CONFIG,(0x00<<3));	//��������,+-250��
	IIC_Write(MPU6050_ADDR,ACCEL_CONFIG,(0x00<<3));	//���ٶȼ�����,+-2g
/*
	IIC_Write(MPU6050_ADDR,0x6B,0x00);
	IIC_Write(MPU6050_ADDR,0x6A,0x00);
	IIC_Write(MPU6050_ADDR,0x37,0x02);
*/
}


void MPU6050_Get(void)
{
	IIC_Read(MPU6050_ADDR,ACCEL_XOUT_H,IIC_buf,14);

	ax = (IIC_buf[0]<<8) + IIC_buf[1] - ax_cl;//X����ٶ�
	ay = (IIC_buf[2]<<8) + IIC_buf[3] - ay_cl;//Y����ٶ�
	az = (IIC_buf[4]<<8) + IIC_buf[5] - az_cl;//Z����ٶ�

	gx = (IIC_buf[8]<<8) + IIC_buf[9] - gx_cl;//X����ٶ�	
	gy = (IIC_buf[10]<<8) + IIC_buf[11] - gy_cl;//Y����ٶ�	
	gz = (IIC_buf[12]<<8) + IIC_buf[13] - gz_cl;//Z����ٶ�	
}


/*
MPU6050У׼����
��IMUˮƽ���ã�z������ʱ������У׼
˼·�Ǽ���N�����ڵ�ƽ��ֵ���õ�У׼����
*/
#define	CL_cnt	128
void MPU6050_calibrate()
{
	unsigned short i;	
	signed int temp[6] = {0};
	for(i=0; i<CL_cnt; i++)
	{
		HAL_Delay(10);
		MPU6050_Get();
		temp[0] += ax;
		temp[1] += ay;
		temp[2] += az;
		temp[3] += gx;
		temp[4] += gy;
		temp[5] += gz;
	}	
	ax_cl = temp[0]/CL_cnt;
	ay_cl = temp[1]/CL_cnt;
	az_cl = temp[2]/CL_cnt - (0xffff>>2); //ƽ��ʱz�����������ٶ�g����ȥgֵ
	gx_cl = temp[3]/CL_cnt;
	gy_cl = temp[4]/CL_cnt;
	gz_cl = temp[5]/CL_cnt;
}

