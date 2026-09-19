from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


source = Path('/tmp/demo1-layout-white-0928.jpg')
target = Path('/tmp/demo1-layout-white-final-status.jpg')

image = Image.open(source).convert('RGB')
draw = ImageDraw.Draw(image)

font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font = ImageFont.truetype(font_path, 28)
label = 'TASK COMPLETE  |  4 TARGETS VERIFIED'

# Keep the completion note on the white canvas, outside the three video panes.
box = (28, 28, 520, 78)
draw.rounded_rectangle(box, radius=8, fill='white', outline=(40, 160, 85), width=2)
draw.text((45, 39), label, font=font, fill=(25, 135, 70))

image.save(target, quality=100, subsampling=0)
