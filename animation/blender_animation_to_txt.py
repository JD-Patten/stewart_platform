import bpy

# Define the cube object
cube = bpy.data.objects["Cube"]

# Open the output file for writing
output_file = open(r"C:\Users\jpatt\Documents\Projects\Blender\cube_animation3.txt", "w")

# Write framerate and number of frames to the output file
output_file.write("Framerate: " + str(bpy.context.scene.render.fps) + "\n")
output_file.write("Number of frames: " + str(bpy.context.scene.frame_end - bpy.context.scene.frame_start + 1) + "\n")


# Loop through every frame in the animation
for frame in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
    
    # Set the current frame
    bpy.context.scene.frame_set(frame)
    
    # Get the location and rotation values of the cube
    location = cube.location
    rotation = cube.rotation_euler
    
    # Write the location and rotation values to the output file
    output_file.write("Frame " + str(frame) + "\n")
    output_file.write("Location: " + str(location) + "\n")
    output_file.write("Rotation: " + str(rotation) + "\n")
    output_file.write("\n")

# Close the output file
output_file.close()