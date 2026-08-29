#!/usr/bin/env python3
"""Create per-floor and summary three-panel room-map diagnostics."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from plot_stage1_3d_truth_comparison import load_hm3d_vertices

def load_pcd(path):
    lines = path.read_bytes().splitlines(); i = next(i for i,x in enumerate(lines) if x.startswith(b"DATA"))
    return np.loadtxt(lines[i+1:], dtype=np.float32, usecols=(0,1,2))

def draw_grid(ax, grid, meta, title):
    colors = np.zeros((*grid.shape, 3), dtype=np.uint8)
    colors[grid < 0] = (170,170,170); colors[grid == 0] = (248,248,248); colors[grid == 100] = (35,35,35)
    o = np.asarray(meta["origin_xy_m"]); r = float(meta["resolution_m"])
    extent = [o[0], o[0]+grid.shape[1]*r, o[1], o[1]+grid.shape[0]*r]
    ax.imshow(np.flipud(colors), extent=extent, origin="lower", interpolation="nearest")
    ax.set_title(title)
    return extent

def add_regions(ax, graph):
    if not graph.exists(): return
    data=json.loads(graph.read_text())
    for room in data.get("rooms", []):
        poly=np.asarray(room.get("polygon_xy_m", []), dtype=float)
        if len(poly)<3: continue
        ax.add_patch(Polygon(poly, closed=True, fill=True, facecolor="#49a078", edgecolor="#17613f", alpha=.18, linewidth=1.5))
        c=np.asarray(room["centroid_xy_m"])
        ax.text(c[0],c[1],f'R{room["id"]}',fontsize=7,ha="center",va="center",alpha=.8)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("run_dir",type=Path); ap.add_argument("multifloor_dir",type=Path); ap.add_argument("scene",type=Path); ap.add_argument("output",type=Path); ap.add_argument("--max-floor",type=int,default=3); args=ap.parse_args()
    manifest=json.loads((args.run_dir/"episode/manifest.json").read_text())
    scene_label = args.scene.parent.name or args.scene.stem
    floors=json.loads((args.multifloor_dir/"floors.json").read_text())[:args.max_floor]
    truth=load_hm3d_vertices(args.scene,manifest); explored=load_pcd(args.run_dir/"bag_export/map_occupied.pcd")
    args.output.mkdir(parents=True,exist_ok=True); panels=[]
    for item in floors:
        n=item["floor"]; z=float(item["floor_z_m"]); band=(z-.12,z+1.8)
        tr=truth[(truth[:,2]>=band[0])&(truth[:,2]<=band[1])]; ex=explored[(explored[:,2]>=band[0])&(explored[:,2]<=band[1])]
        fig,ax=plt.subplots(1,3,figsize=(18,6),sharex=True,sharey=True)
        ax[0].scatter(tr[:,0],tr[:,1],s=.35,c="#222222",alpha=.55,rasterized=True); ax[0].set_title(f"Official HM3D geometry | L{n}")
        point_plot = ax[1].scatter(
            ex[:,0], ex[:,1], s=1.15, c=ex[:,2], cmap="turbo", alpha=.96,
            vmin=band[0], vmax=band[1], linewidths=0, rasterized=True,
        )
        color_axis = inset_axes(ax[1], width="3.2%", height="42%", loc="upper right", borderpad=.8)
        colorbar = fig.colorbar(point_plot, cax=color_axis)
        colorbar.ax.tick_params(labelsize=7)
        ax[1].set_title(f"Explored FALCON point cloud | L{n}")
        st=np.load(args.multifloor_dir/f"L{n}_structure_grid.npy"); sm=json.loads((args.multifloor_dir/f"L{n}_structure_grid.json").read_text()); draw_grid(ax[2],st,sm,f"After Box removal + OccuSG | L{n}"); add_regions(ax[2],args.multifloor_dir/f"L{n}_scene_graph.json")
        for a in ax: a.set_aspect("equal"); a.grid(alpha=.12); a.set_xlabel("FALCON X (m)")
        ax[0].set_ylabel("FALCON Y (m)"); fig.suptitle(f"{scene_label} multi-floor room diagnosis, L{n} z={z:.2f}m"); fig.tight_layout()
        out=args.output/f"L{n}_three_panel.png"; fig.savefig(out,dpi=180,bbox_inches="tight"); plt.close(fig); panels.append(out)
    if panels:
        fig,ax=plt.subplots(len(panels),3,figsize=(18,6*len(panels)),squeeze=False)
        for row,p in enumerate(panels):
            image=plt.imread(p)
            thirds=np.array_split(image,3,axis=1)
            for col,part in enumerate(thirds):
                ax[row,col].imshow(part); ax[row,col].axis("off")
            ax[row,0].set_title(f"L{row+1} | Official truth")
            ax[row,1].set_title(f"L{row+1} | Explored point cloud")
            ax[row,2].set_title(f"L{row+1} | Box-removed + OccuSG")
        floor_label = ", ".join(f"L{x['floor']}" for x in floors)
        fig.suptitle(f"{scene_label} multi-floor room summary ({floor_label})")
        fig.savefig(args.output/"summary.png",dpi=120,bbox_inches="tight"); plt.close(fig)
    print(json.dumps({"floors": [x["floor"] for x in floors], "output": str(args.output), "per_floor": [str(x) for x in panels]},indent=2))
if __name__ == "__main__": main()
