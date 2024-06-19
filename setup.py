from setuptools import setup, find_packages
import codecs
import os.path


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()


with open('requirements.txt') as f:
    requirements = f.read().splitlines()


setup(
    name='t265',
    version='0.1.9',
    description='t265 Tracking camera API wrapper',
    author='Yusuke Tanaka',
    license='LGPLv3',
    project_urls={'GitHub':'https://github.com/Suke0811/Localization_T265'},
    packages=find_packages(include=['t265', 't265.*']),
    install_requires=requirements,
    classifiers=[
        'Development Status :: 4 - Beta',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Programming Language :: Python :: 3',
    ],
)

