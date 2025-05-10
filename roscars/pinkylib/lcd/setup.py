from setuptools import setup, find_packages

setup(
    name="pinky_lcd",
    version="1.0",
    packages=find_packages(),
    install_requires=[
        "rpi-lgpio",
        "spidev",
        "numpy<2.0.0",
    ],
)
