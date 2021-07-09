# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

#### Project Specification
### Path Planning

Project should met specification defined in [project rubric](https://review.udacity.com/#!/rubrics/1971/view)  

| Criteria | Result |
|----------|--------|
|The code compiles correctly| [Done](./images/01_01_build.PNG) |
|The car is able to drive at least 4.32 miles without incident| [Done](./images/01_rubric.PNG) |
|The car drives according to the speed limit | Done |
|Max Acceleration and Jerk are not Exceeded | Done |
|Car does not have collisions | Done |
|The car stays in its lane, except for the time between changing lanes| Done |
|The car is able to change lanes| Done |
|There is a reflection on how to generate paths| See, following section |

### Solution files description

1. All used constants has been moved to [```./src/constants.h```](./src/constants.h)
2. Steps performed according to the class description "2. Getting Started" and "3. More Complex Paths" has been moved to [```./src/steps.h```](./src/steps.h)
3. Spline library that recommended to use for generation smooth trajectory has been downloaded to ```./src/spline.h```

### Solution description

Project task performed in ```do_highway_driving``` function:
1. Process sensor data to detect a obstacles ```process_sensor_fusion```

   Calculate buffer according to the current path size ```end_path_s - car_s``` and some empirically found coefficients 5/4 of reference size in front of car and 3/4 on back side. Lines 93,94 of main.cpp.

2. Update car parameter according to step 1 ```update_lane_and_velocity```

   Looks like in case of driving on highway it can be quite simple. It follow native guess concerning taken lanes and car speed. Lines 133-173 of main.cpp.

3. Generate new paint for future path ```generate_path```

   Implemented according class recommendations. Future steps size ```PATH_FUTURE_STEP_SIZE``` and position ahead ```PATH_POS_AHEAD``` the car has been choose for smooth car moving between lanes. Lines 181-297 of main.cpp.

4. Additional helpers and converter defined at lines 24-45 of main.cpp.


Project has been implemented by Yuriy Ivanov.
