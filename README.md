# A customized version of CRISP



changes to do:

1. ~~remove useless modules/functions from original crisp~~;
2. ~~test if example still works as usual~~; the original software doesn't work actually; I judge
from the output that the result looks ok.
3.


files done:
calibration: ok
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
2. validate the modified version can output the same result
3. compute flow, frame-to-frame rotation, plot
4. rough estimation refinement: 
5. consider if we don't know the readout time, can we estimate
6. ~~check track-retrack plot to see if still large number of wrong tracks exist~~ (**track-retrack is very strong**)
7. ~~save/load slice information, flow~~


## python built-in
list, tuple, range, dict, set
datetime, collections, deque, heapqp, bisect
math -- random, statistics