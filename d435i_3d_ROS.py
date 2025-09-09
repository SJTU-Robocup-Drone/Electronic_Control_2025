import cv2


def detect_black_circle_hsv(image):
    # Convert image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define range for black color in HSV
    lower_black = (0, 0, 0)
    upper_black = (180, 255, 30)

    # Create mask for black color
    mask = cv2.inRange(hsv_image, lower_black, upper_black)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # List to hold center coordinates of detected circles
    centers = []

    for contour in contours:
        # Approximate the contour
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

        # Check if the contour is circular by looking at the number of vertices
        if len(approx) > 8:  # More vertices for more circular shapes
            M = cv2.moments(contour)
            if M['m00'] != 0:
                # Calculate center coordinates
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                centers.append((cX, cY))

    return centers

# Integrate this function into the main detection loop
# Assuming there is a main detection function, you would call it like this:
# centers = detect_black_circle_hsv(image)

# Add additional code to integrate black circle detection with YOLO detection here.