#ArucoBasedLanding

( Do not use the code directly, project is here to guide you only. )

This project aims that with a good calibration, one aruco marker can help the landing of infinite drones.

There are 2 windows when it is executed, one for openCV and ArUco and the other one is for OpenGL. The OpenGL part, it is basically one point visualization of the ArUco and camera. Note that tvecs and rvecs are inverted which means it is not camera centered, it is ArUco centered. ArUco center is not changing which means that you have a constant point when you are landing drone. You just calculate the distances in x,y and z. Thanks to OpenCV DecompositionMatrix, we get yaw, pitch and roll. Yaw is enough for landing but you can use roll and pitch for compensating. 

According to our observations, error rate is at most 2cm and it is totally okay for industrial drones. With normalization it can be 1 cm. For a better success rate, you can calibrate the camera better than me and program the drone so that for landing it moves a little bit slow. It is assumed that there is only one marker. The code is not done yet (29 Aug 2018) so I am still not sure that I should use more than one marker for better accuracy.



camera_calibration.cpp file is taken from OpenCV examples. Use VID5.xml and in_VID5.xml as a parameter. In the file VID5.xml, don't give absolute path!

arucoDetect.cpp is the main file. 







TODO:

- Comments :)

- Restructure the repository so unncesessary files and directories won't be pushed.

- Normalization of the data so drone won't compensate too much.
