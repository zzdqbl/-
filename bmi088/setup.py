from setuptools import find_packages, setup

package_name = 'imu_bmi088'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'bmi088_imu = imu_bmi088.bmi088_imu_node:main',
          'akm_bmi088 = imu_bmi088.akm_bmi088:main',
          'accvel_bmi088 = imu_bmi088.accvel_bmi088:main',
        ],
    },
)
