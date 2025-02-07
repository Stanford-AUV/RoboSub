from setuptools import setup, find_packages
import dvl_utils

setup(
    name="dvl_utils",
    version=dvl_utils.__version__,
    author="trdi",
    author_email="rdifs@teledyne.com",
    url="http://www.teledynemarine.com/rdi/",
    packages=find_packages(),
    python_requires=">=3.6",
    install_requires=["pyserial", "numpy"],
)
