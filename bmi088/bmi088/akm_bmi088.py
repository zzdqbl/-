#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import Hobot.GPIO as GPIO
import spidev

# ======================== 硬件与算法配置 ========================
ACC_RANGE = 12.0
GYRO_RANGE = 2000.0
DT = 0.01
SAMPLING_FREQ = 1.0 / DT

# ====================== 核心：关闭重力修正 ======================
twoKp = 0.0    # 彻底关闭加速度修正
twoKi = 0.0    # 彻底关闭积分
# =================================================================

# 你想固定的初始姿态（水平 Z 向上，强硬锁定）
q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
integralFBx, integralFBy, integralFBz = 0.0, 0.0, 0.0

def InvSqrt(number):
    return 1.0 / math.sqrt(number)

# 纯陀螺仪积分（无重力修正，姿态永远保持你给的初始值）
def MahonyUpdate(gx, gy, gz, ax, ay, az):
    global q0, q1, q2, q3

    # 只积分陀螺仪，完全不看加速度
    gx *= 0.5 * DT
    gy *= 0.5 * DT
    gz *= 0.5 * DT

    q0 += (-q1 * gx - q2 * gy - q3 * gz)
    q1 += (q0 * gx + q2 * gz - q3 * gy)
    q2 += (q0 * gy - q1 * gz + q3 * gx)
    q3 += (q0 * gz + q1 * gy - q2 * gx)

    # 归一化保持稳定
    recipNorm = InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0 *= recipNorm
    q1 *= recipNorm
    q2 *= recipNorm
    q3 *= recipNorm

class BMI088:
    def __init__(self, acc_cs_pin, gyro_cs_pin, spi_bus=1, spi_device=0, spi_speed=5000000):
        self.acc_cs_pin = acc_cs_pin
        self.gyro_cs_pin = gyro_cs_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.acc_cs_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.gyro_cs_pin, GPIO.OUT, initial=GPIO.HIGH)

        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = spi_speed
        self.spi.mode = 0b00

        self.ACC_X_LSB = 0x12
        self.ACC_Y_LSB = 0x14
        self.ACC_Z_LSB = 0x16
        self.GYRO_X_LSB = 0x02
        self.GYRO_Y_LSB = 0x04
        self.GYRO_Z_LSB = 0x06
        
    def read_register(self, reg, cs_pin):
        GPIO.output(cs_pin, GPIO.LOW)
        resp = self.spi.xfer2([reg | 0x80, 0x00])
        GPIO.output(cs_pin, GPIO.HIGH)
        return resp[1]

    def write_register(self, reg, val, cs_pin):
        GPIO.output(cs_pin, GPIO.LOW)
        self.spi.xfer2([reg & 0x7F, val])
        GPIO.output(cs_pin, GPIO.HIGH)

    def initialize(self):
        self.write_register(0x7D, 0x04, self.acc_cs_pin)
        self.write_register(0x7C, 0x02, self.acc_cs_pin)
        time.sleep(0.01)
        self.write_register(0x15, 0x04, self.gyro_cs_pin)
        self.write_register(0x14, 0x01, self.gyro_cs_pin)
        time.sleep(0.01)

    def _read16(self, reg, cs):
        lsb = self.read_register(reg, cs)
        msb = self.read_register(reg + 1, cs)
        val = (msb << 8) | lsb
        if val > 32767:
            val -= 65536
        return val

    def read_accel(self):
        return self._read16(self.ACC_X_LSB, self.acc_cs_pin), \
               self._read16(self.ACC_Y_LSB, self.acc_cs_pin), \
               self._read16(self.ACC_Z_LSB, self.acc_cs_pin)

    def read_gyro(self):
        return self._read16(self.GYRO_X_LSB, self.gyro_cs_pin), \
               self._read16(self.GYRO_Y_LSB, self.gyro_cs_pin), \
               self._read16(self.GYRO_Z_LSB, self.gyro_cs_pin)

    def close(self):
        self.spi.close()
        GPIO.cleanup()

class BMI088ImuNode(Node):
    def __init__(self):
        super().__init__('bmi088_imu_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(DT, self.publish_imu) 

        self.bmi = BMI088(acc_cs_pin=24, gyro_cs_pin=26)
        self.bmi.initialize()

        # ==================== 关键：禁用自动校准 ====================
        # calibrate_initial_orientation(self.bmi)
        # ===========================================================

        self.get_logger().info("✅ BMI088 已启动：姿态强硬固定，无重力牵引")

    def publish_imu(self):
        ax_raw, ay_raw, az_raw = self.bmi.read_accel()
        gx_raw, gy_raw, gz_raw = self.bmi.read_gyro()

        acc_factor = ACC_RANGE / 32768.0
        ax = ax_raw * acc_factor
        ay = ay_raw * acc_factor
        az = az_raw * acc_factor

        gyro_factor = GYRO_RANGE / 32768.0 * (math.pi / 180.0)
        gx = gx_raw * gyro_factor
        gy = gy_raw * gyro_factor
        gz = gz_raw * gyro_factor

        # 姿态更新：纯陀螺仪，无重力修正
        MahonyUpdate(gx, gy, gz, ax, ay, az)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        imu_msg.orientation.w = q0
        imu_msg.orientation.x = q1
        imu_msg.orientation.y = q2
        imu_msg.orientation.z = q3

        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.orientation_covariance[0] = 0.01
        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        self.bmi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BMI088ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()