import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="mirte_robot",
    install_requires=["websocket_server"], # TODO: Require mirte_msgs? rcl_interfaces? controller_manager_msgs?
    # Also requires "rclpy"
    version="0.1.0",
    author="Martin Klomp",
    author_email="m.klomp@tudelft.nl",
    description="Python API for the Mirte Robot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/mirte-robot/mirte-python",
    packages=['mirte_robot'],
    license="Apache License 2.0",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
