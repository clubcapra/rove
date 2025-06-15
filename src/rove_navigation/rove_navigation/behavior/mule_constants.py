# states
START = "START"  # At the beginning, exiting this state sets the startpoint
FOLLOWING = "FOLLOWING"  # make person_following behaviour active
REWIND = "REWIND"  # Initiates the rewind phase
PROCESSED_REWIND = (
    "PROCESSED_REWIND"  # Continues the rewind phase until another TPOSE is detected
)
PAUSE = "PAUSE"  # Freezes the robot in place (TODO: nice to have, make it still oriente itself towards the target, but not move)
END = "END"
REACH_A = "REACH_A"
REACH_B = "REACH_B"

# poses
FIGURE_IDLE = "IDLE"  # Continue state
FIGURE_TPOSE = "TPOSE"  # Start the follow
FIGURE_BUCKET = "BUCKET"  # Pause/UnPause
FIGURE_UP = "UP"  # start the rewind phase
