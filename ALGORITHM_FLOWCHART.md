@startuml
skinparam backgroundColor white

start
:Initialize Hardware, SLAM, & Control Systems;

partition "State: WAIT_START" {
  while (Gesture / Start Signal Detected?) is (No)
    :Wait;
  endwhile (Yes)
}

:Start Vacuum & Enter Boundary Discovery;

partition "State: BOUNDARY_DISCOVERY" {
  repeat
    :Execute Wall Following (Follow Table Edge);
    fork
      :Update Pose Graph (Odom + IMU);
    fork again
      :Fit Rectangle to Edge Points;
    end fork
  repeat while (Lap Complete?) is (No)
  
  :Loop Closure: Match Start/End Node, Optimize Pose Graph, Sync EKF;
}

:Generate Coverage Lanes (Boustrophedon / Grid);

partition "State: COVERAGE" {
  :Enter Coverage Mode;
  while (All Lanes Complete?) is (No)
    :Follow Current Lane (Pure Pursuit);
    
    if (Table Edge Detected?) then (Yes)
      partition "Tactile Localization" {
        :Snap Heading/Pos to Cardinal Wall;
        :Update EKF Constraints;
      }
      :Execute Recovery (Back up & Turn);
    else (No)
      if (Lane Complete?) then (Yes)
        :Advance to Next Lane;
      else (No)
        :Continue Lane;
      endif
    endif
  endwhile (Yes)
}

partition "State: DONE" {
  :Stop Motors & Vacuum, Play Finish Sound;
}

stop
@enduml
