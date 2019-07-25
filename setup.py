import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name = "pyrplidar",
    version = "0.1.1",
    author = "Hyun-je",
    author_email = "bigae2@gmail.com",
    license = "MIT",
    description = "Full-featured python library for Slamtec RPLIDAR series",
    long_description = long_description,
    long_description_content_type = "text/markdown",
    url = "https://github.com/Hyun-je/pyrplidar",
    install_requires = ["pyserial"],
    packages = setuptools.find_packages(),
    py_modules = [
        "pyrplidar",
        "pyrplidar_serial",
        "pyrplidar_protocol"
    ],
    platforms = "Cross Platform",
    classifiers = [
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
