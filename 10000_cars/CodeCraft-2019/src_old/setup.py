from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
ext_modules = [Extension("hello",["CodeCraft2019_4.pyx"])]
setup(
    name = "hello",
    cmdclass = {'build_ext': build_ext},
    ext_modules = ext_modules
)