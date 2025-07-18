from setuptools import setup

package_name = 'vision_rescue_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nonprawich I.',
    maintainer_email='hey@npwitk.com',
    description='Vision processing for rescue robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_rescue_bot.vision_node:main',
	    'camera_simulator = vision_rescue_bot.camera_simulator:main'
        ],
    },
)
