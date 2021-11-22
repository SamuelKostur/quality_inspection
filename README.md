# quality_inspection

catkin make on school PC needs "include" folder, even if its empty

version notes:
    this version should be safe to use when program overide is 100, however i suggest limit program overide for example to 10 for first run after robot instalation
    changed movement type to PTP since it solves problem with:
        1) unpredictable axis speed when singularity (since robot wont get to singularity)
        2) high rotation speed of 4-6 even though the position and orientation of TCP doesnt change much - with limit on VEL_AXIS[]
           that was problem mainly because of cable from camera which could get stuck and damage camera and i couldnt stop it since it would happen to fast
        3) STOP due to software limit switch, when robot cant continue because he has reached maximal axis rotation - this wont happen in PTP

notes:
    VEL_AXIS[] limit doesnt work when using LIN motion
    $VEL.CP and $ACC.CP not used when using PTP motion
