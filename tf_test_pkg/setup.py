from setuptools import find_packages, setup

package_name = 'tf_test_pkg'

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
    maintainer='dar',
    maintainer_email='974782852@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_publisher = tf_test_pkg.tf_publisher:main',
            'tf_subscriber = tf_test_pkg.tf_subscriber:main',
        ],
    },
)
