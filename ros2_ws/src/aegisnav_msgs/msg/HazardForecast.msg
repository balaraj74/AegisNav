# Time-indexed hazard forecast. OccupancyGrid.data is used as normalized risk
# in [0, 100]; -1 means unavailable/unknown. Uncertainty layers use the same
# encoding. The static traversability map remains a separate message.
std_msgs/Header header
float32 time_step_s
nav_msgs/OccupancyGrid[] risk_layers
nav_msgs/OccupancyGrid[] uncertainty_layers
