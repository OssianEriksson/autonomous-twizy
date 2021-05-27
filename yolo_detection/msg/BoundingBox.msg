# Bounding box inferred around an object in an image by an object detection
# algorithm

# Estimated probability of correct classification
float64 probability

# Lower and upper x coordinates (in pixels) of inferred box (lower x values to
# the left in the image)
int16 xmin
int16 xmax

# Lower and upper y coordinates (in pixels) of inferred box (lower y values at
# the top of the image)
int16 ymin
int16 ymax

# Integer id and name of the class of object detected inside the bounding box
int16 id
string Class