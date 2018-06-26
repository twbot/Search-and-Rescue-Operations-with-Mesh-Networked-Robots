from distutils.core import setup

setup(
	version="0.0.0",
	scripts=["src/main.py"],
	packages=["ros_rover_move"],
	package_dir={'':'src'}
	)