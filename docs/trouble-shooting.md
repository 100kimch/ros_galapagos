# Troubleshooting

- authored in 2020. 01. 14.

## Notices

### no module named 'geometry_msgs'

- When making `catkin_ws` (Catkin workspace), check python version before `catkin make`

```bash
python3 --version
Python 3.5.2
catkin make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/libpython3.5m.so
```

### don't forget to add `source devel/setup.bash`

### no module named 'em'

```bash
sudo apt install python3-pip
sudo python3 -m pip install empy
```
