import os
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import can
from ttkthemes import ThemedStyle
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from PIL import Image, ImageTk
import sv_ttk
from tkfontawesome import icon_to_image
import math



root = tk.Tk()
root.title("Arctos CAN controller")

# Global variables
selected_port = None
connected = False
bus = None
last_modified = None
last_joint_states = None
last_cartesian_position = None

class CanvasButton:
    """ Create left mouse button clickable canvas image object.

    The x, y coordinates are relative to the top-left corner of window.
    """

    def __init__(self, canvas, x, y, width, height, image_path, name, command):
        x, y = canvas.canvasx(x), canvas.canvasy(y)  # Convert window to canvas coords.
        self.name = name
        self.canvas = canvas
        self.command = command

        # Open the button image using PIL and convert it to PhotoImage
        self.btn_image = Image.open(image_path)
        self.btn_image = self.btn_image.resize((width, height), Image.LANCZOS)  # Use Lanczos resampling
        self.btn_image = ImageTk.PhotoImage(self.btn_image)

        self.button = canvas.create_image(x, y, anchor='nw', image=self.btn_image, tags=self.name)


def btn_clicked(button_name):
    """ Update slider values based on the clicked canvas button and plan Cartesian path """
    increment = {'X+': 0.03, 'X-': -0.03, 'Y+': -0.03, 'Y-': 0.03, 'Z+': -0.03, 'Z-': 0.03}

    if button_name in increment:
        dx, dy, dz = 0, 0, 0
        if button_name == 'X+':
            dx = increment[button_name]
        elif button_name == 'X-':
            dx = increment[button_name]
        elif button_name == 'Y+':
            dy = increment[button_name]
        elif button_name == 'Y-':
            dy = increment[button_name]
        elif button_name == 'Z+':
            dz = increment[button_name]
        elif button_name == 'Z-':
            dz = increment[button_name]

        # Fetch current slider values
        current_x = cartesian_path_sliders[0].get()
        current_y = cartesian_path_sliders[1].get()
        current_z = cartesian_path_sliders[2].get()

        # Update slider values
        cartesian_path_sliders[0].set(current_x + dx)
        cartesian_path_sliders[1].set(current_y + dy)
        cartesian_path_sliders[2].set(current_z + dz)

        # Plan Cartesian path
        plan_cartesian_path()



def update_joint_sliders(data):
    """ Update joint sliders based on received joint state data """
    global last_joint_states
    # Check if the received joint states are different from the last ones
    if last_joint_states is None or data.position != last_joint_states.position:
        last_joint_states = data
        # Update joint sliders and labels here
        for i, joint in enumerate(data.name):
            if joint in joint_name_to_slider:
                slider = joint_name_to_slider[joint]
                slider.set(data.position[i])

def update_cartesian_sliders(data):
    """ Update Cartesian sliders based on received transformed tf data """
    global last_cartesian_position
    # Check if the received Cartesian position is different from the last one
    if last_cartesian_position is None or \
        data.transform.translation.x != last_cartesian_position.transform.translation.x or \
        data.transform.translation.y != last_cartesian_position.transform.translation.y or \
        data.transform.translation.z != last_cartesian_position.transform.translation.z:
        last_cartesian_position = data
        # Update Cartesian sliders and labels here
        cartesian_path_sliders[0].set(data.transform.translation.x)
        cartesian_path_sliders[1].set(data.transform.translation.y)
        cartesian_path_sliders[2].set(data.transform.translation.z)

# Function to refresh available ports
def refresh_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    port_combobox['values'] = ports
    port_combobox.current(0)  # Select the first port by default

# Function to connect to a selected port
def connect():
    global selected_port, connected, bus
    port = port_combobox.get()

    if not port:
        update_message("Please select a port.")
        return

    try:
        bus = can.interface.Bus(bustype="slcan", channel=port, bitrate=50000)
        connected = True
        selected_port = port
        update_message(f"Connected to port {port}.")
    except Exception as e:
        update_message(f"Error connecting: {str(e)}")

# Function to disconnect from the currently selected port
def disconnect():
    global connected, bus
    if connected:
        try:
            bus.shutdown()
            connected = False
            update_message(f"Disconnected from port {selected_port}.")
        except Exception as e:
            update_message(f"Error disconnecting: {str(e)}")
    else:
        update_message("Not connected to any port.")

# Function to send messages
def send():
    os.system("python3 convert.py")
    os.system("python3 gcode2jog.py")
    os.system("python3 send.py")

# Function to clear messages
def clear_messages():
    messages_text.config(state=tk.NORMAL)
    messages_text.delete('1.0', tk.END)
    messages_text.config(state=tk.DISABLED)

# Function to update messages
def update_message(message):
    messages_text.config(state=tk.NORMAL)
    messages_text.insert(tk.END, message + "\n")
    messages_text.see(tk.END)  # Scroll to the end of the text widget
    messages_text.config(state=tk.DISABLED)

    # Print the message to console for debugging purposes
    print("Message updated:", message)

# Function to run the ROS script
def run_ros_script():
    os.system("python3 rosjog.py")
    os.system("python3 roscan.py")
    os.system("python3 ros.py")

# Function to update joint state value
def update_joint_state_value(_):
    for i, slider in enumerate(joint_state_sliders):
        joint_state_values[i].set("{:.2f}".format(slider.get() * (180 / math.pi)))  # Convert radians to degrees

# Function to update cartesian path value
def update_cartesian_path_value(_):
    for i, slider in enumerate(cartesian_path_sliders):
        cartesian_path_values[i].set("{:.2f}".format(slider.get() * (180 / math.pi)))  # Convert radians to degrees


# Function to go to joint state
def go_to_joint_state():
    joint_state_values = [slider.get() for slider in joint_state_sliders]
    ui_command_pub.publish("go_to_joint_state," + ','.join(map(str, joint_state_values)))

# Function to plan cartesian path
def plan_cartesian_path():
    cartesian_path_values = [slider.get() for slider in cartesian_path_sliders]
    ui_command_pub.publish("plan_cartesian_path," + ','.join(map(str, cartesian_path_values)))

def open_gripper():
    ui_command_pub.publish("open_gripper")

def close_gripper():
    ui_command_pub.publish("close_gripper")

# Function to send the data to gcode.txt
def send_data():
    # Get values from the entry fields
    address = address_entry.get()
    data_fields = [entry.get() for entry in data_entries]
    
    # Combine all values into one string
    combined_data = address + ''.join(data_fields)
    
    # Write combined data to "gcode.txt"
    try:
        with open("gcode.txt", "w") as gcode_file:
            gcode_file.write(combined_data)
        update_message("Data written to 'gcode.txt' successfully.")
    except Exception as e:
        update_message(f"Error writing to 'gcode.txt': {e}")
    os.system("python3 send.py")

# Button icons
toggle = icon_to_image("toggle-on", fill="#646464", scale_to_width=25)
connekt = icon_to_image("plug", fill="#646464", scale_to_width=15)
ref = icon_to_image("sync-alt", fill="#646464", scale_to_width=20)
play = icon_to_image("play", fill="#646464", scale_to_width=15)
stop = icon_to_image("stop", fill="#646464", scale_to_width=15)
er = icon_to_image("eraser", fill="#646464", scale_to_width=20)

ico = Image.open('/home/aa/Desktop/arctosgui/img/icon.png')
photo = ImageTk.PhotoImage(ico)
root.wm_iconphoto(False, photo)

# Togle theme button
toggle_button = ttk.Button(root, image=toggle, command=sv_ttk.toggle_theme)
toggle_button.grid(row=0, column=3, padx=5, pady=5, sticky="e")


# Refresh button
refresh_button = ttk.Button(root, image=ref, command=refresh_ports)
refresh_button.grid(row=0, column=0, padx=5, pady=5, sticky="w")

# Port selection dropdown
port_combobox = ttk.Combobox(root, width=25)
port_combobox.grid(row=0, column=1, padx=5, pady=5)

# Connect button
connect_button = ttk.Button(root, text="Connect", image=connekt, compound=tk.LEFT, command=connect)
connect_button.grid(row=0, column=2, padx=5, pady=5)


# Disconnect button
disconnect_button = ttk.Button(root, text="Disconnect", command=disconnect)
disconnect_button.grid(row=0, column=3, padx=5, pady=5, sticky="w")

# Send button
send_button = ttk.Button(root, text="Run RoboDK", image=play, compound=tk.LEFT, command=send)
send_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="e")

# Stop button
stop_button = ttk.Button(root, text="Stop", image=stop, compound=tk.LEFT,)
stop_button.grid(row=1, column=3, padx=5, pady=5, sticky="w")

# Clear messages button
clear_button = ttk.Button(root, image=er, command=clear_messages)
clear_button.grid(row=3, column=3, padx=5, pady=5, sticky="e")

# Messages display
messages_label = ttk.Label(root, text="Messages:")
messages_label.grid(row=3, column=0, columnspan=4, padx=5, pady=(10, 5), sticky="w")

messages_text = tk.Text(root, height=8, width=50, state=tk.DISABLED)
messages_text.grid(row=4, column=0, columnspan=5, padx=5, pady=(0, 5), sticky="ew")

# Create frame for the inputs
input_frame = ttk.Frame(root)
input_frame.grid(padx=5, pady=0, columnspan=15, sticky="nw")

# Labels for each input field
address_label = ttk.Label(input_frame, text="Address")
address_label.grid(row=5, column=0, padx=5, pady=5)

label_texts = ["Mode", "Speed","", "Acc","", "Position","", "CRC"]
data_labels = [ttk.Label(input_frame, text=label_text) for label_text in label_texts]
for idx, label in enumerate(data_labels):
    label.grid(row=5, column=idx+2, padx=5, pady=5)

# Entry fields for each input
address_entry = ttk.Entry(input_frame, width=2)
address_entry.grid(row=6, column=0, padx=5, pady=5)


data_entries = [ttk.Entry(input_frame, width=2) for _ in range(8)]
for idx, entry in enumerate(data_entries):
    entry.grid(row=6, column=idx+2, padx=5, pady=5)

# Button to send the data
send_button = ttk.Button(input_frame, text="Send", command=send_data)
send_button.grid(row=6, column=12, padx=5, pady=5)

# Label to display messages
message_label = ttk.Label(root, text="")
message_label.grid(pady=10)

# Run ROS button
run_ros_button = ttk.Button(root, text="Run ROS",image=play,compound=tk.LEFT, command=run_ros_script)
run_ros_button.grid(row=1, column=2, padx=5, pady=5)

# Joint State Pose Sliders
joint_state_label = ttk.Label(root, text="Joint jog:")
joint_state_label.grid(row=8, column=0, columnspan=2, padx=5, pady=5, sticky="w")
joint_state_sliders = []
joint_state_values = []
joint_name_to_slider = {}
for i, label in enumerate(['X', 'Y', 'Z', 'A', 'B', 'C']):
    ttk.Label(root, text=label).grid(row=i+9, column=0, padx=5, pady=5, sticky='w')
    slider = ttk.Scale(root, from_=-3.14, to=3.14, orient=tk.HORIZONTAL, length=200, command=update_joint_state_value)
    slider.set(0)
    slider.grid(row=i+9, column=1, padx=5, pady=5)
    joint_state_sliders.append(slider)
    joint_state_values.append(tk.StringVar())
    ttk.Label(root, textvariable=joint_state_values[i]).grid(row=i+9, column=2, padx=5, pady=5)
    joint_name_to_slider[f"joint{i+1}"] = slider

# Cartesian Path Sliders
cartesian_path_label = ttk.Label(root, text="Tool jog:")
cartesian_path_label.grid(row=15, column=0, columnspan=2, padx=5, pady=5, sticky="w")
cartesian_path_sliders = []
cartesian_path_values = []
for i, label in enumerate(['X', 'Y', 'Z']):
    ttk.Label(root, text=label).grid(row=i+16, column=0, padx=5, pady=5, sticky='w')
    slider = ttk.Scale(root, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, length=200, command=update_cartesian_path_value)
    slider.set(0)
    slider.grid(row=i+16, column=1, padx=5, pady=5)
    cartesian_path_sliders.append(slider)
    cartesian_path_values.append(tk.StringVar())
    ttk.Label(root, textvariable=cartesian_path_values[i]).grid(row=i+16, column=2, padx=5, pady=5)

# Create buttons
joint_state_button = ttk.Button(root, text="Move joints", command=go_to_joint_state)
joint_state_button.grid(row=19, column=0, padx=5, pady=(5,10))

cartesian_path_button = ttk.Button(root, text="Move tool", command=plan_cartesian_path)
cartesian_path_button.grid(row=19, column=1, padx=5, pady=(5,10))

# Open Gripper button
open_gripper_button = ttk.Button(root, text="Open Gripper", command=open_gripper)
open_gripper_button.grid(row=19, column=3, padx=5, pady=(5,10), sticky="e")

# Close Gripper button
close_gripper_button = ttk.Button(root, text="Close Gripper", command=close_gripper)
close_gripper_button.grid(row=19, column=3, padx=(5,10), pady=(5,10), sticky="w")

# CanvasButton setup
canvas = tk.Canvas(root, height=302, width=302, bd=0, highlightthickness=0, relief="ridge")
canvas.grid(row=9, column=3, padx=5, pady=5, rowspan=10, sticky="w")  # Span across multiple rows

# Replace these paths with the actual paths to your image files
BACKGROUND_IMAGE_PATH = "/home/aa/Desktop/arctosgui/img/strelice.png"
BUTTON_IMAGE_PATH = "/home/aa/Desktop/arctosgui/img/bg.png"

# Open the background image using PIL and resize it to zoom out
background_img = Image.open(BACKGROUND_IMAGE_PATH)
zoom_level = 0.8  # Adjust the zoom level as needed (0.8 means 80% of original size)
new_width = int(background_img.width * zoom_level)
new_height = int(background_img.height * zoom_level)
background_img = background_img.resize((new_width, new_height), Image.LANCZOS)  # Use Lanczos resampling
background_img = ImageTk.PhotoImage(background_img)
background = canvas.create_image(151, 151, image=background_img)

# Define the coordinates, sizes, and names for 6 buttons
button_info = [
    ((48, 28), 65, 59, "Z+"),
    ((185, 28), 65, 59, "Z-"),
    ((110, 100), 73, 40, "Y-"),
    ((12, 145), 79, 46, "X-"),
    ((200, 145), 79, 46, "X+"),
    ((95, 200), 100, 70, "Y+")
]

# Create buttons for each location
buttons = []
for (x, y), width, height, name in button_info:
    button = CanvasButton(canvas, x, y, width, height, BUTTON_IMAGE_PATH, name, btn_clicked)
    buttons.append(button)

def on_click(event):
    x, y = event.x, event.y
    for button in buttons:
        if x >= button.canvas.coords(button.button)[0] and \
           y >= button.canvas.coords(button.button)[1] and \
           x <= button.canvas.coords(button.button)[0] + button.btn_image.width() and \
           y <= button.canvas.coords(button.button)[1] + button.btn_image.height():
            button.command(button.name)
            break

root.bind("<Button-1>", on_click)

# ROS initialization
rospy.init_node('arctos_control_gui', anonymous=True)
ui_command_pub = rospy.Publisher('/ui_command', String, queue_size=10)
joint_states_sub = rospy.Subscriber('/joint_states', JointState, update_joint_sliders)
transformed_tf_sub = rospy.Subscriber('/transformed_tf', TransformStamped, update_cartesian_sliders)

# Configure row and column weights for resizing
for i in range(20):
    root.rowconfigure(i, weight=1)
for i in range(4):
    root.columnconfigure(i, weight=1)

# Set theme light/dark
sv_ttk.set_theme("light")

root.mainloop()
