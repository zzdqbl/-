#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import Hobot.GPIO as GPIO
import spidev

# 传感器配置
ACC_RANGE = 6.0
GYRO_RANGE = 2000.0
DT = 0.005  # 200Hz

class BMI088:
    def __init__(self, acc_cs_pin, gyro_cs_pin, spi_bus=1, spi_device=1, spi_speed=5000000):
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
        # ACC: ±12g, 200Hz
        self.write_register(0x7D, 0x04, self.acc_cs_pin)
        self.write_register(0x7C, 0x01, self.acc_cs_pin)
        time.sleep(0.01)

        # GYRO: ±2000dps, 200Hz
        self.write_register(0x15, 0x04, self.gyro_cs_pin)
        self.write_register(0x14, 0x01, self.gyro_cs_pin)
        time.sleep(0.01)

    def _read16(self, reg, cs):
        lsb = self.read_register(reg, cs)
        msb = self.read_register(reg + 1, cs)
        val = (lsb << 8) | msb
        if val > 32767:
            val -= 65536
        return val

    def _read16_(self, reg, cs):
        lsb = self.read_register(reg, cs)
        msb = self.read_register(reg + 1, cs)
        val = ( msb<< 8) | lsb
        if val > 32767:
            val -= 65536
        return val

    def read_accel(self):
        return self._read16(self.ACC_X_LSB, self.acc_cs_pin), \
               self._read16(self.ACC_Y_LSB, self.acc_cs_pin), \
               self._read16(self.ACC_Z_LSB, self.acc_cs_pin)

    def read_gyro(self):
        return self._read16_(self.GYRO_X_LSB, self.gyro_cs_pin), \
               self._read16_(self.GYRO_Y_LSB, self.gyro_cs_pin), \
               self._read16_(self.GYRO_Z_LSB, self.gyro_cs_pin)

    def close(self):
        self.spi.close()
        GPIO.cleanup()

class BMI088ImuNode(Node):
    def __init__(self):
        super().__init__('bmi088_imu_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.bmi = BMI088(acc_cs_pin=24, gyro_cs_pin=26, spi_bus=1, spi_device=0)
        self.bmi.initialize()

        # XY零偏校准
        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0
        self.calibrate_imu()

        self.timer = self.create_timer(DT, self.publish_imu)
        self.get_logger().info("BMI088 原始数据节点启动成功 ✅")

    def calibrate_imu(self):
        sum_ax = 0
        sum_ay = 0
        sum_az = 0
        n = 100
        self.get_logger().info("正在校准 IMU XY 偏移... 请保持静止")
        for _ in range(n):
            ax, ay, az = self.bmi.read_accel()
            sum_ax += ax
            sum_ay += ay
            sum_az += az
            time.sleep(0.01)
        self.ax_offset = sum_ax / n
        self.ay_offset = sum_ay / n
        self.az_offset = sum_az / n
        self.get_logger().info(f"校准完成：ax_offset={self.ax_offset:.2f}, ay_offset={self.ay_offset:.2f}, az_offset={self.az_offset:.2f}")

    def publish_imu(self):
        # ======================
        # 读取原始值
        # ======================
        ax_raw, ay_raw, az_raw = self.bmi.read_accel()
        gx_raw, gy_raw, gz_raw = self.bmi.read_gyro()

        # XY去零偏
        ax_raw_cal = ax_raw 
        ay_raw_cal = ay_raw - self.ay_offset
        az_raw_cal = az_raw - self.az_offset
        # ======================
        # 单位转换（标准ROS单位）
        # ======================
        acc_scale = ACC_RANGE / 32768.0 * 9.81
        gyr_scale = GYRO_RANGE / 32768.0 * math.pi / 180.0

        ax = az_raw_cal * acc_scale
        ay = ay_raw_cal * acc_scale
        az = ax_raw_cal * acc_scale

        gx = gx_raw * gyr_scale
        gy = gy_raw * gyr_scale
        gz = gz_raw * gyr_scale

        # ======================
        # ROS2 标准打印（核心！）
        # ======================
        self.get_logger().info("=====================================================")
        self.get_logger().info(f"原始加速度 : ax={ax_raw:6.0f}, ay={ay_raw:6.0f}, az={az_raw:6.0f}")
        self.get_logger().info(f"校准加速度 : ax={ax:8.2f}, ay={ay:8.2f}, az={az:8.2f} m/s²")
        self.get_logger().info(f"原始陀螺仪 : gx={gx_raw:6.0f}, gy={gy_raw:6.0f}, gz={gz_raw:6.0f}")
        self.get_logger().info(f"校准陀螺仪 : gx={gx:8.3f}, gy={gy:8.3f}, gz={gz:8.3f} rad/s")
        self.get_logger().info("=====================================================")

        # ======================
        # 发布ROS消息
        # ======================
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # 无姿态解算，设为默认值
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

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