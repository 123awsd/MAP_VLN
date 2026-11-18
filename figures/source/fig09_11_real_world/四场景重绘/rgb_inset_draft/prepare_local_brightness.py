"""Use generated segmentation alpha only; retain source photographic RGB pixels."""
from pathlib import Path
import json
from PIL import Image, ImageFilter, ImageChops, ImageStat, ImageDraw

BASE = Path(__file__).resolve().parent
ASSETS = BASE / 'assets'
SPECS = {
    '0_1': ('enhanced_0_1.png', .88),
    '0_2': ('display_0_2_enhanced.png', .68),
    '0_5': ('enhanced_0_5.png', .72),
    '1_1': ('inset_1_1.jpg', .82),
    '1_4': ('inset_1_4.jpg', .72),
    '1_5': ('inset_1_5.jpg', .78),
    '2_5': ('backpack_reference_inset.jpg', .80),
}

report = []
for key, (filename, gamma) in SPECS.items():
    source = Image.open(ASSETS / filename).convert('RGB')
    cutout = Image.open(ASSETS / f'segmentation_{key}.png').convert('RGBA')
    # Discard low-opacity background remnants; only alpha, never generated RGB,
    # enters the final rendering. Feather by less than a source pixel.
    alpha = cutout.getchannel('A').point(lambda v: 255 if v >= 128 else 0)
    alpha = alpha.resize(source.size, Image.Resampling.LANCZOS)
    if key == '2_5':
        # The cutout mistook the pool-table pocket rim and a loose tabletop
        # strap for part of the backpack. Restrict to its reviewed visible body.
        keep = Image.new('L', source.size, 0)
        points = [(0, 0), (1200, 0), (1200, 650), (894, 650),
                  (886, 705), (870, 735), (842, 750), (800, 742),
                  (780, 735), (0, 735)]
        ImageDraw.Draw(keep).polygon([(round(x*source.width/1200), round(y*source.height/900))
                                     for x, y in points], fill=255)
        alpha = ImageChops.multiply(alpha, keep)
    alpha = alpha.filter(ImageFilter.GaussianBlur(.55))
    alpha.save(ASSETS / f'target_mask_{key}.png')
    lut = [round(255*(v/255)**gamma) for v in range(256)]
    lifted = source.point(lut*3)
    output = Image.composite(lifted, source, alpha)
    output.save(ASSETS / f'local_bright_{key}.png')
    # A separate graphic overlay follows the generated segmentation contour.
    # It adds no reconstructed object pixels and does not replace the photo.
    outline_width = 624
    outline_height = round(outline_width*source.height/source.width)
    silhouette = alpha.resize((outline_width, outline_height), Image.Resampling.LANCZOS)
    silhouette = silhouette.point(lambda v: 255 if v >= 128 else 0)
    radius = 20  # approximately 1 pt after placement in the manuscript
    expanded = silhouette.filter(ImageFilter.MaxFilter(2*radius+1))
    ring = ImageChops.subtract(expanded, silhouette)
    # A narrow dark outer edge preserves contrast over bright photo backgrounds.
    halo_radius = radius+6
    halo_expanded = silhouette.filter(ImageFilter.MaxFilter(2*halo_radius+1))
    halo_ring = ImageChops.subtract(halo_expanded, silhouette)
    outline = Image.new('RGBA', silhouette.size, (12, 35, 48, 0))
    outline.putalpha(halo_ring)
    blue = Image.new('RGBA', silhouette.size, (128, 240, 255, 0))
    blue.putalpha(ring)
    outline = Image.alpha_composite(outline, blue)
    outline.save(ASSETS / f'outline_{key}.png')
    # Background outside the softened mask must remain bit-identical.
    delta = ImageChops.difference(output, source)
    outside = alpha.point(lambda v: 255 if v == 0 else 0)
    assert Image.composite(delta, Image.new('RGB', source.size), outside).getbbox() is None
    report.append(dict(key=key, source=filename, segmentation=f'segmentation_{key}.png',
                       mask=f'target_mask_{key}.png', output=f'local_bright_{key}.png',
                       gamma=gamma, background_outside_mask_unchanged=True,
                       outline_overlay=f'outline_{key}.png', outline_color='#80F0FF',
                       outline_width_fraction=radius/outline_width,
                       outline_outer_edge_color='#0C2330',
                       outline_outer_edge_width_fraction=(halo_radius-radius)/outline_width,
                       generated_rgb_used=False, mask_fraction=ImageStat.Stat(alpha).mean[0]/255))
(BASE / 'local_brightness_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print('Saved seven locally brightened targets; background preservation verified.')
