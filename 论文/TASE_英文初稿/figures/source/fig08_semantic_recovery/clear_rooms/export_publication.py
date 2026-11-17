"""Export the accepted Fig. 8 composition for the T-ASE manuscript."""
from pathlib import Path
import cairo
import compose_bsc as fig

OUT=Path(__file__).resolve().parent
STEM=OUT/'fig8_household_check_en'
svg=fig.build(publication=True)
STEM.with_suffix('.svg').write_text(svg)

width_pt=184/25.4*72
scale=width_pt/fig.W
surface=cairo.PDFSurface(str(STEM.with_suffix('.pdf')),width_pt,fig.H*scale)
surface.restrict_to_version(cairo.PDF_VERSION_1_5)
surface.set_metadata(cairo.PDF_METADATA_TITLE,'Fig. 8. Semantic recovery during household inspection')
ctx=cairo.Context(surface);ctx.scale(scale,scale)
handle=fig.Rsvg.Handle.new_with_flags(fig.Rsvg.HandleFlags(1))
handle.write(svg.encode());handle.close()
vp=fig.Rsvg.Rectangle();vp.x=vp.y=0;vp.width=fig.W;vp.height=fig.H
handle.render_document(ctx,vp)
surface.finish()
fig.render(svg,STEM.with_suffix('.png'),184/25.4*600/fig.W)

caption=('Illustration of semantic recovery within a multi-object inspection task. '
    'The sequence checks a kettle on the kitchen counter, searches the dining chairs '
    'for a cushion, and checks a bedside lamp. After local reinspection yields no '
    'cushion observation, the recovery module proposes the sofa in the eastern living room as an '
    'alternative search anchor. The cushion is then located, and the mission '
    'continues to the bedroom. Flags associate key observation poses with the '
    'trajectory, top-down maps, and RGB views.')
(OUT/'caption.tex').write_text('\\caption{'+caption+'}\n')
print(STEM.with_suffix('.pdf'))
