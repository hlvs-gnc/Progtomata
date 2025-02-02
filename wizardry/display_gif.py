import tkinter as tk
from PIL import Image, ImageTk, ImageSequence
import sys

class GIFViewer:
    def __init__(self, root, gif_path):
        self.root = root
        self.root.title("Wizard Spell")
        self.root.geometry("440x415")  # Adjust window size if needed
        self.root.attributes("-topmost", True)  # Keep the window always on top

        # Open GIF
        self.gif = Image.open(gif_path)
        self.frames = [ImageTk.PhotoImage(img) for img in ImageSequence.Iterator(self.gif)]

        # Create Label to display the GIF
        self.label = tk.Label(root)
        self.label.pack()
        self.index = 0

        self.update_frame()

    def update_frame(self):
        """ Loop through the GIF frames """
        self.index = (self.index + 1) % len(self.frames)
        self.label.config(image=self.frames[self.index])
        self.root.after(100, self.update_frame)  # Adjust speed as needed

if len(sys.argv) < 2:
    print("Usage: python show_gif.py <path_to_gif>")
    sys.exit(1)

gif_path = sys.argv[1]

# Create the window and run GIF animation
root = tk.Tk()
viewer = GIFViewer(root, gif_path)
root.mainloop()
