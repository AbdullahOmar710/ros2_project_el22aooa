from setuptools import find_packages, setup

ros2_project_el22aooa = 'ros2_project_el22aooa'

setup(
    name=ros2_project_el22aooa,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + ros2_project_el22aooa]),
        ('share/' + ros2_project_el22aooa, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cscajb',
    maintainer_email='x.wang16@leeds.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_step = ros2_project_el22aooa.first_step:main',
            'second_step = ros2_project_el22aooa.second_step:main',
            'third_step = ros2_project_el22aooa.third_step:main',
            'fourth_step = ros2_project_el22aooa.fourth_step:main',
            'combined = ros2_project_el22aooa.combined:main',
            'ros_project = ros2_project_el22aooa.ros_project:main',
        ],
    },
)
