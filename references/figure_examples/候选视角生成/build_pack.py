"""Build a self-contained, attributed Fig.6 reference pack from reviewed crops."""
from pathlib import Path
import csv, hashlib, html, io, json, re
import fitz
from PIL import Image
from curation import PAPERS, RAW, GROUPS

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'
INK=(.12,.16,.21); MUTED=(.35,.40,.46); ACCENT=(.08,.38,.46)

def put(page, rect, text, size=12, color=INK):
    page.insert_font(fontname='CJK', fontfile=FONT)
    rc = page.insert_textbox(fitz.Rect(rect), text, fontname='CJK', fontsize=size,
                             lineheight=1.35, color=color)
    if rc < -0.01: raise ValueError(f'text overflow {rc}: {text[:80]}')

def add_image(page, rect, path, max_pixels=2400):
    im=Image.open(path).convert('RGB'); im.thumbnail((max_pixels,max_pixels), Image.Resampling.LANCZOS)
    b=io.BytesIO(); im.save(b, format='JPEG', quality=95, subsampling=0, optimize=True)
    page.insert_image(fitz.Rect(rect), stream=b.getvalue(), keep_proportion=True)

def save(doc, path):
    doc.subset_fonts(); doc.save(path, garbage=4, deflate=True)

def main():
    (ROOT/'figures').mkdir(exist_ok=True); (ROOT/'overviews').mkdir(exist_ok=True)
    manifest={x['id']:x for x in json.loads((ROOT/'papers.json').read_text())}
    docs={}; papers={}
    for name, meta in PAPERS.items():
        path=ROOT/'papers'/f'{name}.pdf'
        if not path.exists(): raise FileNotFoundError(path)
        doc=fitz.open(path); docs[name]=doc
        row=dict(manifest.get(name, {'id':name, 'url':meta['url']}))
        stamp=re.search(r'arXiv:\s*(\d+\.\d+v\d+)', doc[0].get_text())
        row['url']=('https://arxiv.org/pdf/'+stamp.group(1)) if stamp else meta['url']
        row.update(title=meta['title'], task=meta['task'], habitat=meta['habitat'], evidence=meta['evidence'],
                   pages=len(doc), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   local_pdf=f'papers/{name}.pdf', version=stamp.group(1) if stamp else '来源 PDF')
        papers[name]=row

    entries=[]
    for num, (eid,paper,figure,pageno,clip,group,priority,title,borrow,limit) in enumerate(RAW,1):
        src=docs[paper]; rect=fitz.Rect(clip); page=src[pageno-1]
        if not page.rect.contains(rect): raise ValueError((eid,paper,pageno,clip,page.rect))
        stem=f'{eid}_{paper}_Fig{figure}'.replace('/','-')
        folder=ROOT/'figures'/group; folder.mkdir(exist_ok=True)
        png=folder/f'{stem}.png'; pdf=folder/f'{stem}.pdf'
        pix=page.get_pixmap(matrix=fitz.Matrix(3,3), clip=rect, alpha=False); pix.save(png)
        one=fitz.open(); out=one.new_page(width=rect.width,height=rect.height); out.show_pdf_page(out.rect,src,pageno-1,clip=rect); save(one,pdf)
        v=papers[paper]
        entries.append(dict(id=f'{num:02d}', paper=paper, figure='Fig. '+figure, page=pageno, clip=list(clip),
            group=group, group_title=GROUPS[group], priority=priority, title=title, borrow=borrow,
            limitation=limit, task=v['task'], habitat=v['habitat'], source_url=v['url']+f'#page={pageno}',
            source_sha256=v['sha256'], png=png.relative_to(ROOT).as_posix(), pdf=pdf.relative_to(ROOT).as_posix(),
            source_pdf=v['local_pdf'], panel={'grounding':'a','region':'a','sampling':'b','feasibility':'b/c','selection':'d'}[group], reference_type='视觉表达参考；算法差异见 limitation', checked='对照原 PDF 页面查看图体，裁剪排除正文与图注'))

    pack=fitz.open()
    for e in entries:
        p=pack.new_page(width=1000,height=730)
        put(p,[35,20,965,58],f"{e['id']}  {e['paper']} · {e['figure']}  |  {e['title']}",18)
        put(p,[35,64,965,102],f"PDF 第 {e['page']} 页 · 优先级 {e['priority']} · {e['task']} · Habitat：{e['habitat']}",10,MUTED)
        p.draw_rect(fitz.Rect(35,107,965,523),color=(.86,.89,.91),fill=(1,1,1)); add_image(p,[43,115,957,515],ROOT/e['png'])
        put(p,[35,538,965,595],'可借鉴：'+e['borrow'],12)
        put(p,[35,598,965,654],'与 Fig.6 的区别：'+e['limitation'],11,MUTED)
        put(p,[35,673,965,701],'来源：'+e['source_url'],9,ACCENT)
        p.insert_link({'kind':fitz.LINK_URI,'from':fitz.Rect(35,673,965,701),'uri':e['source_url']})
        put(p,[35,706,965,727],f"{e['group_title']} · 原图版权归论文作者 / 出版方",8,MUTED)
    pack.set_toc([[1,f"{e['id']} {e['paper']} {e['figure']} — {e['title']}",i+1] for i,e in enumerate(entries)])
    full=ROOT/f'Fig6_完整参考图集_{len(entries)}项.pdf'; save(pack,full)

    top_ids=['02','05','06','08','10','13','16','19','21','23','24','25']
    tops=[next(e for e in entries if e['id']==x) for x in top_ids]
    top=fitz.open()
    for e in tops: top.insert_pdf(pack,from_page=entries.index(e),to_page=entries.index(e))
    top.set_toc([[1,f"{e['id']} {e['paper']} {e['figure']}",i+1] for i,e in enumerate(tops)])
    save(top,ROOT/'Fig6_优先12项详解.pdf')

    overview=fitz.open(); sections=[('优先看这 12 项',tops)]+[(GROUPS[g],[e for e in entries if e['group']==g]) for g in GROUPS]
    overview_png=[]
    for title,arr in sections:
        for st in range(0,len(arr),12):
            arr2=arr[st:st+12]; p=overview.new_page(width=1200,height=1260)
            put(p,[28,20,1170,58],'Fig.6 参考图 · '+title,23)
            put(p,[28,64,1170,93],'编号对应完整图集、CSV 和 sources.json；点击缩略图可打开论文来源。',10,MUTED)
            for k,e in enumerate(arr2):
                x=28+(k%3)*392; y=103+(k//3)*281
                p.draw_rect(fitz.Rect(x,y,x+376,y+268),color=(.85,.88,.90),fill=(1,1,1)); add_image(p,[x+8,y+8,x+368,y+187],ROOT/e['png'],900)
                put(p,[x+9,y+193,x+369,y+220],f"{e['id']}  {e['paper']} · {e['figure']} · p{e['page']}",11)
                put(p,[x+9,y+222,x+369,y+263],e['title'],10,MUTED)
                p.insert_link({'kind':fitz.LINK_URI,'from':fitz.Rect(x,y,x+376,y+268),'uri':e['source_url']})
            fn=f'{len(overview):02d}_{re.sub(r"[^0-9A-Za-z_一-龥]+","_",title)}_{st//12+1}.png'; out=ROOT/'overviews'/fn
            p.get_pixmap(matrix=fitz.Matrix(1.35,1.35),alpha=False).save(out); overview_png.append(out.name)
    save(overview,ROOT/'Fig6_分类速览.pdf')

    (ROOT/'sources.json').write_text(json.dumps(dict(research_date='2026-10-24', paper_count=len(papers), reference_units=len(entries),
        papers=papers, figures=entries),ensure_ascii=False,indent=2),encoding='utf-8')
    with (ROOT/'参考图索引.csv').open('w',encoding='utf-8-sig',newline='') as f:
        cols=['id','paper','figure','page','panel','reference_type','group_title','priority','title','task','habitat','borrow','limitation','source_url','png','pdf']
        w=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');w.writeheader();w.writerows(entries)

    lines=['# Fig.6 分方向参考图（重筛版）','',
        f'当前保留 **{len(papers)} 篇论文、{len(entries)} 张不重复参考图**，按五条细分方向整理。旧版图集和未入选资料已移除。',
        '',f'[优先12项](Fig6_优先12项详解.pdf) · [完整图集]({full.name}) · [分类速览](Fig6_分类速览.pdf) · [筛选图库](index.html) · [CSV](参考图索引.csv)',
        '', '## 按 Fig.6 的视觉任务选择','',
        '|细分方向|对应面板|要画清什么|重点参考|', '|---|---|---|---|',
        '|语言与关系|a 的输入|table 是已知 anchor；under 是关系；backpack 是待验证目标|SOON Fig.1/2；ConceptGraphs Fig.1/2|',
        '|观察区域|a 的输出|由 anchor 几何和关系构造区域，再放 region samples；目标位置不能画成已知|VLMaps Fig.1/2；VoxPoser Fig.2/3|',
        '|候选位姿|b|距离×方位角采样、yaw 朝向区域、固定 pitch|FUEL Fig.4；Bubble Explorer Fig.4；MA-SCVP Fig.4|',
        '|约束与可见性|b 侧视 inset、c|高度区间求交、FoV 包含、位置 clearance、样本 LOS 分开解释|ConstrainedSpaces Fig.4/5/16；SEE Fig.8；FALCON Fig.5|',
        '|评分与压缩|d|每方位先按可见点数/代价选代表，再按预算保留角度分散的视角|SCONE Fig.2；MA-SCVP Fig.9；SEE Fig.10/11|',
        '', '## 推荐的组合与限制','',
        '主图仍用同一张桌下场景贯穿 a–d；每组参考仅解决一个局部表达问题。不要把五组论文各自的完整系统框图拼成 Fig.6。',
        '语义关系使用 SOON 的词语与实体同色标记；区域使用 VoxPoser 的透明三维体；采样使用 FUEL 的环向点；约束用侧视 FoV 与独立 pass/fail；筛选用可见样本矩阵和稀疏保留点。以上是针对本项目的绘图建议，不是这些论文共同实现的算法。',
        '局部缺口：目前没有找到与本项目“固定 pitch 的高度交集 + 每方位按可见点数/终端代价排序 + 最小角距扩散”完全一致的单张论文图。高度交集和角度多样性应根据自己的公式原创绘制，不能把 set cover 说成等价算法。',
        '', '## 裁图与来源','',
        'PNG/PDF 裁剪只保留图体及图内标签；长 caption 和正文留在原论文。每张图有原图号、PDF 页序、裁剪坐标、固定版本（可识别时）与源文件 SHA-256。原论文放在 papers/，分方向裁图放在 figures/。',
        'REVERIE、SOON 使用 Matterport3D Simulator，不能因此标作 Habitat。VLMaps 有 Habitat 实验，但本包所选 Fig.1/2 是实景机器人示例。其余选图主要来自主动视觉、覆盖规划和操作领域；选择依据是局部画法与 Fig.6 的相关性。',
        '所有图的版权归原作者/出版方。本包仅作研究与构图参考，未改动项目论文或 Fig.6。',
        '', '## 删除与保留标准','',
        '删除纯网络架构、轨迹结果、失败恢复、普通导航拓扑更新，以及无法对应本图局部机制的价值地图。保留能够单独说明语义关系、空间区域、相机几何、可见性或覆盖互补性的图。ConceptGraphs 只作上游关系表示参考，不扩展 Fig.6 为建图方法图。',
        '', '## 优先12项','']
    for e in tops: lines.append(f"- **{e['id']} {e['paper']} {e['figure']}**：{e['title']}（对应面板 {e['panel']}）。")
    for g,gt in GROUPS.items():
        lines += ['',f'## {gt}','', '|编号|论文/图号|页|优先级|借鉴点|','|---|---|---:|---:|---|']
        for e in entries:
            if e['group']==g: lines.append(f"|{e['id']}|[{e['paper']} {e['figure']}]({e['png']}) · [原论文]({e['source_url']})|{e['page']}|{e['priority']}|{e['title']}|")
    lines += ['', '## 每项图注','']
    for e in entries: lines += [f"### {e['id']} · {e['paper']} {e['figure']} — {e['title']}",'',f"[PNG]({e['png']}) · [裁剪 PDF]({e['pdf']}) · [来源 PDF 第 {e['page']} 页]({e['source_url']})",'',e['borrow'],'','**区别：** '+e['limitation'],'']
    (ROOT/'README.md').write_text('\n'.join(lines),encoding='utf-8')

    cards=[]
    for e in entries:
        esc=html.escape; search=' '.join([e['paper'],e['title'],e['task'],e['habitat'],e['group_title']])
        cards.append(f'''<article data-group="{e['group']}" data-priority="{e['priority']}" data-search="{esc(search)}"><a href="{e['png']}" target="_blank"><img loading="lazy" src="{e['png']}" alt="{esc(e['paper']+' '+e['figure'])}"></a><h3>{e['id']} · {esc(e['paper'])} {e['figure']}</h3><p class="muted">PDF p{e['page']} · 面板 {e['panel']} · 优先级 {e['priority']} · {esc(e['group_title'])}<br>Habitat：{esc(e['habitat'])}</p><b>{esc(e['title'])}</b><p>{esc(e['borrow'])}</p><p class="muted">区别：{esc(e['limitation'])}</p><p><a href="{e['source_url']}" target="_blank">原论文</a> · <a href="{e['pdf']}">裁剪 PDF</a> · <a href="{e['source_pdf']}#page={e['page']}">本地 PDF</a></p></article>''')
    opts=''.join(f'<option value="{g}">{g} · {html.escape(t)}</option>' for g,t in GROUPS.items())
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Fig.6 候选视角生成参考图库</title><style>body{{font-family:system-ui,sans-serif;background:#f3f5f6;color:#22303d;margin:0;padding:30px}}header{{max-width:1120px}}a{{color:#116b79}}.controls{{position:sticky;top:0;background:#f3f5f6;padding:14px 0;display:flex;gap:12px;flex-wrap:wrap;z-index:2}}input,select{{padding:10px;border:1px solid #bccbd1;border-radius:6px;font-size:14px}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:20px}}article{{background:#fff;padding:18px;border:1px solid #dce3e6;border-radius:8px}}article img{{width:100%;height:230px;object-fit:contain}}h3{{margin:14px 0 8px}}p{{line-height:1.55;font-size:14px}}.muted{{color:#5b6874}}article[hidden]{{display:none}}</style><header><h1>Fig.6「候选视角生成」论文参考图库</h1><p>{len(papers)} 篇论文 · {len(entries)} 个参考单元 · 2026-10-24。重点覆盖目标区域、候选采样、FoV/遮挡/碰撞筛选与互补视角。</p><p><a href="README.md">调研说明</a> · <a href="Fig6_优先12项详解.pdf">优先12项</a> · <a href="Fig6_分类速览.pdf">分类速览</a> · <a href="Fig6_完整参考图集_{len(entries)}项.pdf">完整图集</a></p></header><div class="controls"><input id="q" placeholder="检索论文、FoV、遮挡、Habitat…"><select id="g"><option value="">全部主题</option>{opts}</select><select id="p"><option value="">全部优先级</option><option value="1">优先级 1</option><option value="2">优先级 2</option><option value="3">优先级 3</option></select><span id="count"></span></div><main>{''.join(cards)}</main><script>const q=document.querySelector('#q'),g=document.querySelector('#g'),p=document.querySelector('#p');function filter(){{let n=0;document.querySelectorAll('article').forEach(a=>{{const s=(!g.value||a.dataset.group===g.value)&&(!p.value||a.dataset.priority===p.value)&&a.dataset.search.toLowerCase().includes(q.value.toLowerCase());a.hidden=!s;n+=s}});document.querySelector('#count').textContent=n+' 项'}}[q,g,p].forEach(e=>e.addEventListener('input',filter));filter()</script></html>'''
    (ROOT/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps({'papers':len(papers),'reference_units':len(entries),'full_pdf':full.name,'overview_pages':overview.page_count},ensure_ascii=False))

if __name__=='__main__': main()
