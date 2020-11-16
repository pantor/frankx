import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


with open('README.md', 'r') as readme_file:
    long_description = readme_file.read()


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                'CMake must be installed to build the following extensions: ' +
                ', '.join(e.name for e in self.extensions)
            )

        cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
        if cmake_version < LooseVersion('3.11.0'):
            raise RuntimeError('CMake >= 3.11.0 is required')

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        build_type = os.environ.get('BUILD_TYPE', 'Release')
        build_args = ['--config', build_type]

        # Local
        cmake_args += ['-DREFLEXXES_TYPE=ReflexxesTypeII']
        cmake_args += ['-DReflexxes_INCLUDE_DIR=/code/RMLTypeII/include/RMLTypeII/']
        cmake_args += ['-DReflexxes_LIB_DIR=/code/RMLTypeII/build']
        cmake_args += ['-DEIGEN3_INCLUDE_DIRS=/usr/local/include/eigen3/']
        cmake_args += ['-DCMAKE_CXX_FLAGS_RELEASE=-O3']

        # CI
        # cmake_args += ['-DCMAKE_CXX_FLAGS_RELEASE=-O3']
        # cmake_args += ['-DBUILD_PYBIND11=OFF']
        # cmake_args += ['-DUSE_PYTHON_EXTENSION=OFF']
        # cmake_args += ['-DPYBIND11_TEST=OFF']
        # cmake_args += ['-DREFLEXXES_TYPE=ReflexxesTypeII']
        # cmake_args += ['-DReflexxes_INCLUDE_DIR=RMLTypeII/include/RMLTypeII/']
        # cmake_args += ['-DReflexxes_LIB_DIR=RMLTypeII/build']

        # Local
        # cmake_args += ['-DEIGEN3_INCLUDE_DIRS=/usr/local/include/eigen3/']
        # cmake_args += ['-DCMAKE_CXX_FLAGS_RELEASE=-O3']
        # cmake_args += ['-DREFLEXXES_TYPE=ReflexxesTypeIV']
        # cmake_args += ['-DReflexxes_ROOT_DIR=$HOME/Documents/libs/ReflexxesTypeIV']


        # Pile all .so in one place and use $ORIGIN as RPATH
        cmake_args += ['-DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE']
        cmake_args += ['-DCMAKE_INSTALL_RPATH={}'.format('$ORIGIN')]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + build_type]
        build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', '--target', ext.name] + build_args, cwd=self.build_temp)


setup(
    name='frankx',
    version='0.1.0',
    description='High-Level Motion Library for the Franka Panda Robot',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Lars Berscheid',
    author_email='lars.berscheid@kit.edu',
    url='https://github.com/pantor/frankx',
    packages=find_packages(),
    license='LGPL',
    ext_modules=[CMakeExtension('_frankx')],
    cmdclass=dict(build_ext=CMakeBuild),
    keywords=['robot', 'robotics', 'trajectory-generation', 'motion-control'],
    classifiers=[
        'Development Status :: 2 - Pre-Beta',
        'Intended Audience :: Science/Research',
        'Topic :: Software Development :: Build Tools',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Programming Language :: C++',
    ],
    python_requires='>=3.5',
)
