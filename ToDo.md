###   list of things to do in next release ###


20190905 v1.0: changes to make in future release
1. create custom msg with sensor_msgs::Range and covariance and output covariance with relationship as mentioned in datasheet on how accuracy changes with range and measuring mode.
2. convert to nodelet verison
3. add service call for zeroing range measurement
4. modify the JHlidarLite lib for more functionality as mentioned in manual and garmin github page for arduino
5. adding sort of median filter to account for sudden spikes in range measurment and smoothen out measurements.
6. adding readme on how to use and ros wiki page
7. (optional) add urdf with cad model
8. add diagnostic publisher
9. add filtered output
10. add ablitiy to do insitu calibration or calculate co-eff for filter by running a diff node while static above 1m.
