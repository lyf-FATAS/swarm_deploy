from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 自动解析 package.xml 中的元数据
d = generate_distutils_setup(
    packages=['drone_observer'],      # 替换为您的 Python 模块名
    package_dir={'': 'scripts'}            # 指定 Python 代码目录
)

setup(**d)