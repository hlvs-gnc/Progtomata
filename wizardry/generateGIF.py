from PIL import Image, ImageDraw, ImageFont

#    ⁺₊⁺₊    
# ⋆⁺₊✧✩₊˚.⋆
# ✩₊˚.⋆☾⋆⁺₊✧⁺~ = ,. =
#   ⁺₊⁺₊⁺₊

# Load a proper fixed-width font
try:
  font = ImageFont.truetype("DejaVuSansMono.ttf", 14)  # Common monospace font
except IOError:
  font = ImageFont.load_default()  # Fallback to system default

# Function to render ASCII frames with proper formatting
def create_ascii_image(ascii_frame, image_size=(600, 700), font=font):
  img = Image.new("RGB", image_size, "black")
  draw = ImageDraw.Draw(img)
  
  # Fine-tune the text positioning
  line_height = 16  # Adjusted for monospace characters
  x, y = 20, 20  # Left margin and top margin

  for line in ascii_frame:
    draw.text((x, y), line, font=font, fill="white")
    y += line_height  # Ensuring proper vertical spacing

  return img

# Improved ASCII wizard animation with a more dynamic spell effect

ascii_frames = [
    [
        "-------------------------------------------------------",
        "                       PROGTOMATA",
        "-------------------------------------------------------",
        "                                                       ",
        "                             _,-'|",
        "                          ,-'._  |",
        "               .||,       |####\\ |", 
        r"              \.`',/      \####| |",
        "              * ⁺₊⁺ *      |###| |", 
        r"              / || \     ,-'\#/,'`.",
        "                ||     ,'   `,,. `.",
        "                ,|____,' , ,;' \\| |",
        "              (3|\\    _/|/'   _| |",
        "              (S||/,-''  | >-'' _,\\\\",
        "                ||'      ==\\ ,-'  ,'",
        "                ||       |  V \\ ,|",
        "                ||       |    |` |",
        "                ||       |    |   \\",
        "                ||       |    \\    \\",
        "                ||       |     |    \\",
        "                ||       |      \\_,-'",
        "                ||       |___,,--\")_\\",
        "                ||         |_|   ccc/",
        "                ||        ccc/ BDDDD",
        "                ||      BDDDD",
        "                                                       ",
        "_______________________________________________________",
    ],
    # Frame 2 - Expanding spell effect
    [
        "-------------------------------------------------------",
        "                       PROGTOMATA",
        "-------------------------------------------------------",
        "                                                       ",
        "                             _,-'|",
        "                          ,-'._  |",
        "               .||,       |####\\ |", 
        r"              -.`',\      \####| |",
        "             * ₊⁺✩⁺₊ *     |###| |", 
        r"              \ || -     ,-'\#/,'`.",
        "                ||     ,'   `,,. `.",
        "                ,|____,' , ,;' \\| |",
        "              (3|\\    _/|/'   _| |",
        "              (S||/,-''  | >-'' _,\\\\",
        "                ||'      ==\\ ,-'  ,'",
        "                ||       |  V \\ ,|",
        "                ||       |    |` |",
        "                ||       |    |   \\",
        "                ||       |    \\    \\",
        "                ||       |     |    \\",
        "                ||       |      \\_,-'",
        "                ||       |___,,--\")_\\",
        "                ||         |_|   ccc/",
        "                ||        ccc/ BDDDD",
        "                ||      BDDDD",
        "                                                       ",
        "_______________________________________________________",
    ],
    # Frame 3 - Stronger magic surge
    [
        "-------------------------------------------------------",
        "                       PROGTOMATA",
        "-------------------------------------------------------",
        "                                                       ",
        "                             _,-'|",
        "                          ,-'._  |",
        "               .||,       |####\\ |", 
        r"              /.`',-      \####| |",
        "            * ⁺₊✧⁺~= *     |###| |", 
        r"              - || /     ,-'\#/,'`.",
        "                ||     ,'   `,,. `.",
        "                ,|____,' , ,;' \\| |",
        "              (3|\\    _/|/'   _| |",
        "              (S||/,-''  | >-'' _,\\\\",
        "                ||'      ==\\ ,-'  ,'",
        "                ||       |  V \\ ,|",
        "                ||       |    |` |",
        "                ||       |    |   \\",
        "                ||       |    \\    \\",
        "                ||       |     |    \\",
        "                ||       |      \\_,-'",
        "                ||       |___,,--\")_\\",
        "                ||         |_|   ccc/",
        "                ||        ccc/ BDDDD",
        "                ||      BDDDD",
        "                                                       ",
        "_______________________________________________________",
    ]
]

def create_and_crop_ascii_image(ascii_frame, image_size=(600, 700), font=font):
  img = Image.new("RGB", image_size, "black")
  draw = ImageDraw.Draw(img)

  # Set text positioning
  x, y = 20, 20  # Left and top margin
  line_height = 16

  for line in ascii_frame:
    draw.text((x, y), line, font=font, fill="white")
    y += line_height

  # Convert to grayscale and find bounding box of non-black pixels
  gray_img = img.convert("L")
  bbox = gray_img.getbbox()

  # Crop if bounding box exists
  return img.crop(bbox) if bbox else img

# Generate cropped images for each ASCII frame
ascii_images = [create_and_crop_ascii_image(frame) for frame in ascii_frames]

# Save as GIF
output_gif_path = "wizardry/wizard_spell.gif"
ascii_images[0].save(
    output_gif_path,
    save_all=True,
    append_images=ascii_images[1:],
    duration=250,
    loop=0
)

print(f"GIF saved as {output_gif_path}")