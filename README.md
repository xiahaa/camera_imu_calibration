# A customized version of CRISP



changes to do:

1. ~~remove useless modules/functions from original crisp~~;
2. ~~test if example still works as usual~~; the original software doesn't work actually; I judge
from the output that the result looks ok.
3.


files done:
~~calibration: ok~~
~~timesync: ok~~
~~znccpyr.py~~
~~stream.py: ok~~
~~camera.py: ok~~
~~imu_new.py： ok~~
~~ransac.py: ok~~
~~rotations.py: ok~~
~~tracking.py: ok~~
~~videoslice.py: ok~~

## things to notice：
1. readout is not used for projection/backprojection.
2. readout is finally used for optimization.

## todo
1. numpy, scipy 
2. ~~validate the modified version can output the same result~~: seems correct
3. ~~compute flow, frame-to-frame rotation, plot~~
**readout is only useful for final optimization.**
6. ~~check track-retrack plot to see if still large number of wrong tracks exist~~ (**track-retrack is very strong**)
7. ~~save/load slice information, flow~~
8. ~~use adaptive ransac~~
9. ~~padding or just skip~~: no siginificant effect
10. ~~improve tracking via check mean distance~~
10. ~~seems initialization is very important~~: yes, especially for time_offset and gyro freq, only difference at 1e-2 scale
11. customized to my personal usage: 1. ~~load data~~; 2. ~~understand data using matlab~~; ~~angle2dcm and dcm2angle ok.~~
12. ~~checked with gopro data parsing, using gopro imu gyro could synchronized gopro and gps;~~ 
13. ~~find out why vision cannot work for synchronization,~~ (1. is that possible to use gyro mag: answer is yes, so should check the cross correlation part). find out matlab xcorr can find correct delay while znccpyr and coarse-to-fine python version cannot. rely on manual rough alignment.
14. ~~steup mexopencv and opencv c++;~~
15. ~~should work out a matlab version of crisp for automatic synchronization.~~
12. ~~**rough estimation refinement: todo**~~
5. ~~**consider if we don't know the readout time, can we estimate**: ~~

## python built-in
list, tuple, range, dict, set
datetime, collections, deque, heapqp, bisect
math -- random, statistics
os: files and structures.
io: 
time: 
argparse:
csv:
no need to memorize, use as [reference](https://docs.python.org/zh-cn/3/library/index.html)

### logging
```
import logging
logging.basicConfig(filename='xx.log',filemode='w',format="%(asctime)s-%(name)s-%(levelname)s: %(message)s",level=logging.DEBUG)

""" an example
# create logger
logger = logging.getLogger('simple_example')
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
```
**made a example config file that can be used**
can be used just like printf(__FILE__, __LINE__) in c++

### warning
warnings.warn(message)
message could be format as
'this is a warning {:d} - {:f}'.format(a,b)

### future
always add
from __future__ import print_function, division, absolute_import

### threading
lock, semaphore, queue
