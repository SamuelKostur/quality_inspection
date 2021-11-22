# quality_inspection

catkin make on school PC needs "include" folder, even if its empty<br />

version notes:
   - main contains posistions for phoxi scanner calibration
   - before moving robot to each positions clicking enter is needed

previos version notes:<br />
   - this version should be safe to use when program overide is 100, however i suggest limit program overide for example to 10 for first run after robot instalation<br />
   - changed movement type to PTP since it solves problem with:<br />
        1) unpredictable axis speed when singularity (since robot wont get to singularity)<br />
        2) high rotation speed of 4-6 even though the position and orientation of TCP doesnt change much - with limit on VEL_AXIS[]<br />
           that was problem mainly because of cable from camera which could get stuck and damage camera and i couldnt stop it since it would happen to fast<br />
        3) STOP due to software limit switch, when robot cant continue because he has reached maximal axis rotation - this wont happen in PTP<br />

notes:<br />
   - VEL_AXIS[] limit doesnt work when using LIN motion<br />
   - $VEL.CP and $ACC.CP not used when using PTP motion<br />
