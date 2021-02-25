#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/gloria/catkin_ws/src/push_toycar"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/gloria/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/gloria/catkin_ws/install/lib/python2.7/dist-packages:/home/gloria/catkin_ws/build/push_toycar/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/gloria/catkin_ws/build/push_toycar" \
    "/usr/bin/python2" \
    "/home/gloria/catkin_ws/src/push_toycar/setup.py" \
    egg_info --egg-base /home/gloria/catkin_ws/build/push_toycar \
    build --build-base "/home/gloria/catkin_ws/build/push_toycar" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/gloria/catkin_ws/install" --install-scripts="/home/gloria/catkin_ws/install/bin"
