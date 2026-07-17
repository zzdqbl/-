from setuptools import setup

package_name = 'ylc_nav'

setup(
    
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ylc_main = ylc_nav.ylc_main:main',
            'find_goal = ylc_nav.find_goal:main',
            'start_button_node = ylc_nav.start_button_node:main',
        ],
    },
    zip_safe=False,
)