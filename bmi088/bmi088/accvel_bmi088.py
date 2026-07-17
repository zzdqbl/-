#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import Hobot.GPIO as GPIO
import spidev
from collections import deque
import numpy as np

# 硬件配置
ACC_CS     = 24
GYRO_CS    = 26
SPI_BUS    = 1
SPI_DEV    = 1
SPI_MODE   = 0b11

# 参数配置
ACC_RANGE  = 12.0
GYRO_RANGE = 2000.0
DT         = 0.01
ALPHA      = 0.98
FILTER_WINDOW = 8

class BMI088:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(ACC_CS, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(GYRO_CS, GPIO.OUT, initial=GPIO.HIGH)
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEV)
        self.spi.max_speed_hz = 5000000
        self.spi.mode = SPI_MODE

    def write(self, reg, val, cs):
        GPIO.output(cs, GPIO.LOW)
        self.spi.xfer2([reg & 0x7F, val])
        GPIO.output(cs, GPIO.HIGH)
        time.sleep(0.005)

    def init(self):
        self.write(0x7B, 0x10, ACC_CS)
        time.sleep(0.01)
        self.write(0x7C, 0x02, ACC_CS)
        self.write(0x7D, 0x04, ACC_CS)
        self.write(0x15, 0x04, GYRO_CS)
        self.write(0x14, 0x01, GYRO_CS)
        time.sleep(0.02)

    def read_acc(self):
        GPIO.output(ACC_CS, GPIO.LOW)
        d = self.spi.xfer2([0x12|0x80]+[0]*6)
        GPIO.output(ACC_CS, GPIO.HIGH)
        def conv(lo, hi):
            val = (hi << 8) | lo
            return val - 65536 if val > 32767 else val
        return conv(d[1],d[2]), conv(d[3],d[4]), conv(d[5],d[6])

    def read_gyro(self):
        GPIO.output(GYRO_CS, GPIO.LOW)
        d = self.spi.xfer2([0x02|0x80]+[0]*6)
        GPIO.output(GYRO_CS, GPIO.HIGH)
        def conv(lo, hi):
            val = (hi << 8) | lo
            return val - 65536 if val > 32767 else val
        return conv(d[1],d[2]), conv(d[3],d[4]), conv(d[5],d[6])

    def close(self):
        self.spi.close()
        GPIO.cleanup()

class IMUNode(Node):
    def __init__(self):
        super().__init__('bmi088_imu_node')
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(DT, self.imu_cb)
        self.bmi = BMI088()
        self.bmi.init()
        # 仅校准X、Y偏移，Z不校准不清零
        self.calib_xy_only()
        # 滑动滤波队列
        self.fx = deque([0.0]*FILTER_WINDOW, maxlen=FILTER_WINDOW)
        self.fy = deque([0.0]*FILTER_WINDOW, maxlen=FILTER_WINDOW)
        self.fz = deque([0.0]*FILTER_WINDOW, maxlen=FILTER_WINDOW)
        # 四元数初始化
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.get_logger().info("BMI088 IMU 启动成功，仅XY校准、Z轴不偏移清零")

    def calib_xy_only(self):
        """只做X、Y静止校准，Z轴保留原始重力不做偏移扣除"""
        calib_samples = 200
        sum_x, sum_y = 0.0, 0.0
        for _ in range(calib_samples):
            x, y, _ = self.bmi.read_acc()
            sum_x += x
            sum_y += y
            time.sleep(0.01)
        # 只保存XY偏移，Z偏移置0不扣除
        self.off_x = sum_x / calib_samples
        self.off_y = sum_y / calib_samples
        self.off_z = 0.0

    def imu_cb(self):
        raw_x, raw_y, raw_z = self.bmi.read_acc()
        gx_raw, gy_raw, gz_raw = self.bmi.read_gyro()

        # 单位转换 + 仅XY减偏移，Z不减
        ax = (raw_x - self.off_x) * ACC_RANGE / 32768.0
        ay = (raw_y - self.off_y) * ACC_RANGE / 32768.0
        az = raw_z * ACC_RANGE / 32768.0

        # 陀螺仪转rad/s
        gx = gx_raw * GYRO_RANGE / 32768.0 * math.pi / 180.0
        gy = gy_raw * GYRO_RANGE / 32768.0 * math.pi / 180.0
        gz = gz_raw * GYRO_RANGE / 32768.0 * math.pi / 180.0

        # 原坐标系修正
        final_ax =  ay
        final_ay = -ax
        final_az =  az

        # 滑动平均滤波
        self.fx.append(final_ax)
        self.fy.append(final_ay)
        self.fz.append(final_az)
        filt_ax = sum(self.fx) / FILTER_WINDOW
        filt_ay = sum(self.fy) / FILTER_WINDOW
        filt_az = sum(self.fz) / FILTER_WINDOW

        # 互补滤波姿态解算
        qw, qx, qy, qz = self.q
        half_dt = DT * 0.5
        dqw = (-qx*gx - qy*gy - qz*gz) * half_dt
        dqx = ( qw*gx + qy*gz - qz*gy) * half_dt
        dqy = ( qw*gy - qx*gz + qz*gx) * half_dt
        dqz = ( qw*gz + qx*gy - qy*gx) * half_dt

        qw += dqw
        qx += dqx
        qy += dqy
        qz += dqz

        # 归一化
        norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        qw /= norm; qx /= norm; qy /= norm; qz /= norm
        self.q = np.array([qw, qx, qy, qz])

        # 构造ROS Imu消息
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz

        # 转m/s²
        imu_msg.linear_acceleration.x = filt_ax * 9.81
        imu_msg.linear_acceleration.y = filt_ay * 9.81
        imu_msg.linear_acceleration.z = filt_az * 9.81

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        self.pub.publish(imu_msg)

    def destroy_node(self):
        self.bmi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()