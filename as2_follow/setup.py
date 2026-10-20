from setuptools import setup
setup(name='as2_follow', version='0.1.0', packages=['as2_follow'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/as2_follow']),
                  ('share/as2_follow', ['package.xml'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['follow = as2_follow.bridge:main']})
