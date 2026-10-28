# One AprilTag in one image (design.md 10): omni_slam's tag_detector on the robot, the sim's bridge in the sim
int32 id                      # 36h11 id
float32 size                  # m, the black square (from the tag map, or the family's default)
float64[8] corners            # undistorted pixels u0 v0 .. u3 v3, counter-clockwise from the tag's bottom left
float32 margin                # the decoder's decision margin (its confidence)
int32 hamming                 # bits corrected
