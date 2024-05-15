def copy_replace(gcode_file, jog_file):
    # Open the gcode file and read its contents
    with open(gcode_file, 'r') as gcode:
        gcode_lines = gcode.readlines()

    # Extract the second row, removing 'G90 ' if present
    second_row = gcode_lines[1].replace('G90 ', '')

    # Open the jog file and read its contents
    with open(jog_file, 'r') as jog:
        jog_lines = jog.readlines()

    # Replace the entire third row in jog file with second row from gcode file
    jog_lines[2] = second_row + '\n'

    # Write the modified jog file
    with open(jog_file, 'w') as jog:
        jog.writelines(jog_lines)

# Example usage
gcode_file = 'gcode.tap'
jog_file = 'jog.tap'
copy_replace(gcode_file, jog_file)
print("Replacement completed successfully.")

