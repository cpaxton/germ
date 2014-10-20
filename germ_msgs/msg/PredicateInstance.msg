
# No header information! This is stored in the PredicateInstanceList

# =====================
# Provide a defined predicate
# Connect two objects
# Predicates are directed from parent to child:
# LEFT-OF(parent, child) means child is LEFT-OF parent.
# =====================
germ_msgs/Predicate predicate
germ_msgs/Object parent
germ_msgs/Object child
germ_msgs/Properties data

# is this bidirectional?
bool is_bidirectional

# describe what we should do with this predicate
byte operation
byte ADD=1
byte REMOVE=2
