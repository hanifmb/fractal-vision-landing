# fractal-vision-landing

`drone_controller` package allows for an accurate landing to a visual target Fractal Marker for any multirotor running Arducopter.

## Installation

In addition to the provided packages, `mavros` and `video_stream_opencv` are needed to provide a communication driver and real-time image feed.

```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget <https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh>
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

apt-get install ros-kinetic-video-stream-opencv
```

The packages in this repository can be built by simply running `catkin_make` or `cakin build` at the root workspace directory.

## Usage

The multirotor needs a predefined waypoint uploaded to the FCU, excluding the take-off command before starting the mission. 

To start the mission, ensure that the RC channel-7 is set to low. Then run `roslaunch vision_landing mission.launch`, and turn channel-7 to high to trigger the mission. After the mission is started, the camera will find the Fractal Marker along the pathway before attempting to land on it. 

<p align="center">
  
<img src="https://user-images.githubusercontent.com/40484370/155521020-ca529201-9cef-4cd2-815b-46b41fcd317d.jpg" width="325"/> 
<img src="https://user-images.githubusercontent.com/40484370/155521065-3d89870a-1e07-434c-adf1-daefa977a240.jpg" width="325"/> 

</p>

To kill the vision landing thread at any time during the mission, turn channel-7 back to low; the multirotor will then remain in Loiter mode and can manually be taken over by the pilot. To modify the bound channel, please refer to this part of code (currently hardcoded):
[https://github.com/villainjoe/fractal-vision-landing/blob/7878cd8869687798c383fc49029a7d810c1c0629/vision_landing/src/DroneController.cpp#L307](https://github.com/villainjoe/fractal-vision-landing/blob/7878cd8869687798c383fc49029a7d810c1c0629/vision_landing/src/DroneController.cpp#L307)


