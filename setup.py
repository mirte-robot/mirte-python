import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="zoef_python",
    install_requires=["websocket_server", "rospkg"],
    version="0.1.0",
    author="Martin Klomp",
    author_email="m.klomp@tudelft.nl",
    description="Python API for the Zoef Robot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/zoef-robot/zoef_python",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
