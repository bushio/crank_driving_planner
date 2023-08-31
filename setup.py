from setuptools import setup
import os
import glob
package_name = 'crank_driving_planner'
submodules = [
    'crank_driving_planner/bound_checker',
    'crank_driving_planner/config',
    'crank_driving_planner/predictor',
    'crank_driving_planner/util']
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name] + submodules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ishibushi Satoshi',
    maintainer_email='s.ishibushi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crank_driving_planner_node = crank_driving_planner.Crank_driving_planner:main',
        ],
    },
)
