from distutils.core import setup,Extension

#MOD = 'Extest' #模块名
#setup(name=MOD,ext_modules=[Extension(MOD,sources=['test_ex_py.cpp'])]) #源文件名


from distutils.core import setup, Extension

module1 = Extension('Extest',
                    sources = ['test_ex_py.cpp'])

setup (name = 'PackageName',
       version = '1.0',
       description = 'This is a demo package',
       ext_modules = [module1])