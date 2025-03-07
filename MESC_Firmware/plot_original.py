import base64
import re
import sys
import time
import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import queue
import threading
from matplotlib.widgets import Slider, Button
import csv
import matplotlib
matplotlib.rcParams["toolbar"] = "toolmanager"
from matplotlib.backend_tools import ToolBase

# Regular expressions
osc52_pattern = re.compile(r'\033]52;c;(.*?)\a')
label_pattern = re.compile(r'\b([a-zA-Z_][a-zA-Z0-9_]*)\b(?=\s*[fuiFUIHh][+-]?\d*\.?\d*)')
type_pattern = re.compile(r'\b([fuiFUIHh])([+-]?\d*\.?\d*)?\b')
bits_pattern = re.compile(r'<header (\d+)bits>')


# Queue to handle incoming datasets
data_queue = queue.Queue()


def follow_osc52_chunks(file_path, chunk_size=1024):
    """Continuously read OSC52 chunks from a file as they are appended."""
    with open(file_path, "rb") as file:  # Open file in binary mode
        buffer = b""  # Initialize a buffer to hold incomplete data

        while True:
            # Read the next chunk of data
            chunk = file.read(chunk_size)
            if not chunk:  # If no new data, wait briefly and try again
                #time.sleep(0.01)
                plt.pause(0.001)
                continue

            # Add the chunk to the buffer
            buffer += chunk

            # Extract complete OSC52 sequences from the buffer
            while True:
                match = osc52_pattern.search(buffer.decode('utf-8', errors='ignore'))
                if not match:
                    break

                # Extract and yield the complete sequence
                coded_osc52_data = match.group(1)
                buffer = buffer[match.end():]  # Remove processed data from the buffer

                #print(coded_osc52_data)

                try:
                    # Decode Base64 encoded data
                    decoded_osc52_data = base64.b64decode(coded_osc52_data)
                    yield decoded_osc52_data
                except (base64.binascii.Error, UnicodeDecodeError) as e:
                    print(f"Error decoding OSC52 data: {e}")

            # Retain any incomplete data in the buffer (already handled above)

# Configuration
CHUNK_SIZE = 2000  # Number of points to display at a time

# Initialize data storage
time_data = []
value_data = {}
time = 0

# Matplotlib setup
fig = ax = 0
lines = {}
ani = 0
labels = []
is_constructed = False
labels_changed = False

# Slider axis
slider_ax = 0
slider = 0
span_slider_ax = 0
span_slider = 0

# Save as csv button
save_button_ax = 0
save_button = 0

mylock = threading.Lock()

# Initialize the graph
def initGraph():
    global value_data
    global lines

    # Store line visibility state
    mylock.acquire()
    vis_list = []
    if labels_changed:
        legend = ax.get_legend()
        legend_lines = legend.get_lines()  # Get the legend line handles
        for leg_line in legend_lines:
            orig_line = lines[leg_line.get_label()]
            visible = orig_line.get_visible()
            vis_list.append(visible)

    # Clear the existing plot
    ax.clear()
    time_data.clear()

    value_data = {label: [] for label in labels} # Dictionary to store y-values for each label (excluding 'time')
    lines = {label: ax.plot([], [], label=label)[0] for label in labels}
    
    ax.set_xlabel("Time")
    ax.set_ylabel("Values")

    # Add legend
    legend = ax.legend()
    legend_lines = legend.get_lines()  # Get the legend line handles
    for i, leg_line in enumerate(legend_lines):
        leg_line.set_picker(True)  # Make legend lines pickable

        # Restore line visibility state
        if i < len(vis_list):
            if not vis_list[i]:
                orig_line = lines[leg_line.get_label()]
                orig_line.set_visible(False)

                # Dim the legend text
                leg_line.set_alpha(0.2)
    mylock.release()

    return lines.values()

# Function to update the graph
def update(frame):
    global value_data
    global lines
    global slider_ax
    global slider
    global labels_changed
    global time

    slider.valmax = time+1
    slider_ax.set_xlim(0, time+1)
    slider.ax.figure.canvas.draw_idle()

    start_index = int(slider.val)  # Get the slider's current value
    end_index = start_index + int(span_slider.val)

    while not data_queue.empty():
        # Get the next dataset from the queue
        dataset = data_queue.get()
        #time = dataset[0]
        time_data.append(time)

        # Update the y-values for each label
        for label, value in zip(labels, dataset):
            value_data[label].append(value)
        
        time = time + 1

    if labels_changed:
        initGraph()
        labels_changed = False

    # Update data for each line based on the slider range
    for label in labels:
        if len(value_data[label]) > 0:
            lines[label].set_data(
                time_data[start_index:end_index],
                value_data[label][start_index:end_index],
            )

    ax.relim(visible_only=True)
    ax.autoscale_view()
    return lines.values()

def on_pick(event):

    # Get the line from the legend
    legend_line = event.artist

    # Find the original line corresponding to the legend entry
    orig_line = lines[legend_line.get_label()]

    # Toggle visibility
    visible = not orig_line.get_visible()
    orig_line.set_visible(visible)

    # Dim or highlight the legend text
    legend_line.set_alpha(1.0 if visible else 0.2)

    ax.relim(visible_only=True)  # Recompute data limits
    ax.autoscale_view()  # Rescale axes
    fig.canvas.draw()  # Redraw the canvas

# Custom tool to save data as CSV
class SaveCSVTool(ToolBase):
    """Custom tool to save data as a CSV file."""

    def trigger(self, *args, **kwargs):
        global time_data, value_data, labels

        if not time_data or not value_data:
            print("No data to save!")
            return

        # Define the output file name
        file_name = "output_data.csv"

        try:
            with open(file_name, mode="w", newline="") as file:
                writer = csv.writer(file)
                
                # Write the header
                header = ["Time"] + labels
                writer.writerow(header)
                
                # Write the data
                for i in range(len(time_data)):
                    row = [time_data[i]] + [value_data[label][i] for label in labels]
                    writer.writerow(row)

            print(f"Data saved to {file_name}")
        except Exception as e:
            print(f"Error saving data to CSV: {e}")

def unpack_n_bits(buffer, bit_count, scale, sample_count, max_bits, old_values):
    """
    Unpacks a buffer encoded by sample_n_bits().

    Parameters:
        buffer (bytes): The packed buffer as a byte array.
        bit_count (int): The number of bits used for each sample.
        scale (list): The scaling factors used during encoding.
        sample_count (int): The number of samples to unpack.

    Returns:
        list: The unpacked and scaled values.
    """
    #max_diff = (1 << (bit_count - 1)) - 1  # Max positive value for bit_count bits
    mask = (1 << bit_count) - 1           # Mask for extracting `bit_count` bits

    bit_pos = 0  # Bit position tracker

    while bit_pos < max_bits:

        unpacked_values = []

        for i in range(sample_count):
            # Determine the byte and bit offset
            byte_idx = bit_pos // 8
            bit_offset = bit_pos % 8

            # Read `bit_count` bits from the buffer
            remaining_bits = bit_count
            encoded = 0
            shift = 0

            while remaining_bits > 0:
                # Extract bits from the current byte
                available_bits = 8 - bit_offset
                bits_to_read = min(remaining_bits, available_bits)
                bits = (buffer[byte_idx] >> bit_offset) & ((1 << bits_to_read) - 1)

                # Add the bits to the encoded value
                encoded |= bits << shift

                # Update counters and pointers
                bit_pos += bits_to_read
                remaining_bits -= bits_to_read
                shift += bits_to_read
                byte_idx = bit_pos // 8
                bit_offset = bit_pos % 8

            # Decode the value (two's complement if negative)
            if encoded & (1 << (bit_count - 1)):  # Check the sign bit
                #diff = -((~(encoded-1)))  # Negative value (two's complement)
                diff = -((~encoded & mask) + 1)  # Negative value (two's complement)
            else:
                diff = encoded  # Positive value



            # Decode the value (two's complement if negative)
            #if encoded & (1 << (bit_count - 1)):  # Check the sign bit
            #    diff = -((~encoded & mask) + 1)  # Negative value (two's complement)
            #else:
            #    diff = encoded  # Positive value

            # Scale and reconstruct the value
            diff /= scale[i]
            value = old_values[i] + diff
            old_values[i] = value

            # Append to the list of unpacked values
            unpacked_values.append(value)

        data_queue.put(unpacked_values)


# Example usage
if __name__ == "__main__":

    file_path = sys.argv[1]

    bits = -1
    labels = format_string = scales = old_values = ''
    bitEncoded = False

    i = 0
    for osc52_chunk in follow_osc52_chunks(file_path):

        if b"<header " in osc52_chunk:
            osc52_str = osc52_chunk.decode('utf-8')
            print(osc52_str)

            # Extract bits
            match = re.search(bits_pattern, osc52_str)
            if match:
                bits = int(match.group(1))
            else:
                print("No bits setup found in header!")

            # Extract labels
            labels = re.findall(label_pattern, osc52_str)

            # Extract types and scaling factors
            types_with_scales = re.findall(type_pattern, osc52_str)

            # Extract types and compute scales
            format_string = [t[0] for t in types_with_scales]
            scales = [float(t[1]) if t[1] else 1.0 for t in types_with_scales]

            old_values = [0.0 for _ in labels]

            # Check if any value in scales is different from 1.0
            bitEncoded = any(scale != 1.0 for scale in scales)

            # Output results
            print("Labels:", labels)
            print("Types:", format_string)
            print("Scales:", scales)

            if not is_constructed:
                # Connect the event to the handler
                fig, ax = plt.subplots()
                fig.canvas.manager.set_window_title("MESC data visualizer")
                plt.subplots_adjust(bottom=0.2)

		        # Slider axis
                slider_ax = plt.axes([0.1, 0.1, 0.8, 0.03])
                slider = Slider(slider_ax, "Start Point", 0, 1, valinit=0, valstep=1)
                span_slider_ax = plt.axes([0.1, 0.14, 0.8, 0.03])
                span_slider = Slider(span_slider_ax, "Window", 0, CHUNK_SIZE, valinit=CHUNK_SIZE-1, valstep=1)

                # Add the save button
                #save_button_ax = plt.axes([0.8, 0.02, 0.1, 0.05])  # Position: [left, bottom, width, height]
                #save_button = Button(save_button_ax, "Save CSV")  # Button label
                #save_button.on_clicked(save_data_to_csv)  # Bind the button to the callback function

                tm = fig.canvas.manager.toolmanager
                tm.add_tool("Save CSV", SaveCSVTool)
                fig.canvas.manager.toolbar.add_tool(tm.get_tool("Save CSV"), "toolgroup")

                fig.canvas.mpl_connect("pick_event", on_pick)                
                ani = FuncAnimation(fig, update, init_func=initGraph, blit=False, interval=100)

                plt.show(block=False)
                is_constructed = True
            else:
                labels_changed = True

        else:
            # 4 bytes: float (f)
            # 2 bytes: int16 (h)
            # 2 bytes: uint16 (H)
            # 4 bytes: int32 (i)
            # 4 bytes: uint32 (I)


            if (bitEncoded):

                sample_count = len(labels)
                bits_per_sample = bits * sample_count;
                max_bits = ((len(osc52_chunk)*8) // bits_per_sample) * bits_per_sample

                unpack_n_bits(osc52_chunk, bits, scales, sample_count, max_bits, old_values)
            else:
                dataset = struct.unpack(format_string, osc52_chunk)
                data_queue.put(dataset)

            # Output the extracted values
            #print(dataset)

            if i%30 == 0:
                plt.pause(0.001)  # Pause to update the plot
            i += 1
