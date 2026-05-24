from setuptools import find_packages, setup

package_name = 'map_augmenter'

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
    maintainer='thedoctor',
    maintainer_email='marco.negrete@ingenieria.unam.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_inflater = map_augmenter.map_inflater:main',
            'map_inflater_solved = map_augmenter.map_inflater_solved:main',
            'split_and_merge = map_augmenter.split_and_merge:main',
            'split_and_merge_solved = map_augmenter.split_and_merge_solved:main',
            'gvd = map_augmenter.gvd:main',
            'gvd_solved = map_augmenter.gvd_solved:main',
        ],
    },
)
