from PIL import Image
import numpy as np
from scipy.ndimage import label, binary_dilation

def filter_with_expanded_black(input_file, output_file, dilation_radius=2):
    # Load the PGM file
    img = Image.open(input_file)
    
    # Convert the image to a numpy array for processing
    img_array = np.array(img)
    
    # Threshold the image to identify white regions (255 = white)
    thresholded_image = (img_array == 254).astype(int)

    # Label the connected components in the thresholded image
    labeled_image, num_features = label(thresholded_image)

    # Calculate the area of each region
    region_areas = [np.sum(labeled_image == i) for i in range(1, num_features + 1)]

    # Identify the index of the largest region
    largest_region_index = np.argmax(region_areas)

    # Create an output image where everything is initially grey
    output_image = np.full_like(img_array, 205)  # Set the entire image to grey

    # Find the largest white region
    white_region = (labeled_image == (largest_region_index + 1))

    # Dilate the white region by 5 pixels (create a 5px wide border of black)
    dilated_white_region = binary_dilation(white_region, iterations=dilation_radius)

    # Assign white to the largest region and its dilated black border
    output_image[white_region] = 254  # Set the largest white region to white
    output_image[dilated_white_region & ~white_region] = 0  # Set the expanded black region

    # Convert the output image to a PGM file format
    output_img = Image.fromarray(output_image)
    output_img.save(output_file)

# Example usage with a dilation radius of 5 pixels:
input_path = '/home/capstone/f1host_ws/src/f1tenth_system/f1tenth_stack/maps/7AprilGP.pgm'  # Replace with the path to your input PGM file
output_path = '/home/capstone/f1host_ws/src/f1tenth_system/f1tenth_stack/maps/7AprilGP.pgm'  # Replace with the desired output PGM file path

filter_with_expanded_black(input_path, output_path)
