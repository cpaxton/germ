
# =====================
# Header information
# Stores node ID producing this message, plus timestamp
# =====================
std_msgs/Header header
germ_msgs/SourceInfo info

# =====================
# Provide a defined predicate
# Connect two objects
# =====================
germ_msgs/Predicate predicate
germ_msgs/Object parent
germ_msgs/Object child
germ_msgs/Properties data
