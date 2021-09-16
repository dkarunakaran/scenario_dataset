# boost_ros_pkg
ros package for latest Boost C++ Libraries (http://www.boost.org/).  
this package is not official.  
this package will download source from [https://github.com/boostorg/boost.git](https://github.com/boostorg/boost.git)

this catkin backage build boost library in catkin workspace,  
and enable to `pkg-config [OPTION] boost_ros_pkg`.

# How to Build

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rkoyama1623/boost_ros_pkg.git
cd boost_ros_pkg
catkin build --this
```
# About Makefile.boost
```bash
make -f Makefile.boost install.boost PREFIX=$HOME/local
# generate
# $HOME/local/include/boost
# $HOME/local/lib/libboost*
```

## options
- MODULE=*module name*  
build modular boost

```
make -f Makefile.boost install.boost PREFIX=$HOME/local MODULE=python # build only boost/python
```

