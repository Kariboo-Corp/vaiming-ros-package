from setuptools import setup

package_name = 'px100_pointing_controller_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mathis',
    maintainer_email='mreinert@bordeaux-inp.fr',
    description='Controller to point to a target using mobile_px100 Interbotix arm',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px100_pointing_node = px100_pointing_controller_py.pointing_controller:main'
        ],
    },
)
