# =====================
# Header information
# Stores node ID producing this message, plus timestamp
# =====================
std_msgs/Header header
germ_msgs/SourceInfo info

# =====================
# List of predicates sent
# =====================
germ_msgs/PredicateInstance[] predicates

# is this a diff, or all the world predicates?
bool is_diff
