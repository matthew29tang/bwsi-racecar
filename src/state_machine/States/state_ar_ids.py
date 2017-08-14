class STATE_AR_IDS:
    # ids to get out of current states
    RollingWeave_ID = 19 # -> go to overpass
    RollingWeave_dist_des = 10

    Overpass_ID =  20 #-> go to water hazard
    Overpass_dist_des = 0.5

    Water_ID =  21 #-> go to underpass
    Water_dist_des = 0.5

    Underpass_ID = 19 #-> go to mesh wall
    Underpass_dist_des = 0.7

    MeshWall_ID = 16 # -> FINISH
    MeshWall_dist_des = 0.75
