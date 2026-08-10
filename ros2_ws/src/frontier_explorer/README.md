# frontier_explorer

ROS 2 frontier-based exploration node for Stage 1 mapping.

It subscribes to /map, computes frontiers, and sends NavigateToPose goals.
It starts on /mission/control=start and stops on /mission/control=end by default.
