"""Build an attributed, offline figure reference library from reviewed crops."""
from pathlib import Path
import csv
import hashlib
import html
import json
import re
import io
import fitz
from PIL import Image
from curation import PAPERS, RAW, GROUPS

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'
INK=(.12,.16,.21)
MUTED=(.35,.40,.46)
ACCENT=(.10,.40,.47)

def box(page, rect, text, size=12, color=INK):
    page.insert_font(fontname='CJK',fontfile=FONT)
    result=page.insert_textbox(fitz.Rect(rect),text,fontname='CJK',fontsize=size,lineheight=1.35,color=color)
    if result < -0.01:
        raise ValueError(f'Text overflow ({result}): {text[:80]}')

def photo(page,rect,path,max_pixels=2500):
    im=Image.open(path).convert('RGB')
    im.thumbnail((max_pixels,max_pixels),Image.Resampling.LANCZOS)
    data=io.BytesIO()
    im.save(data,format='JPEG',quality=95,subsampling=0,optimize=True)
    page.insert_image(fitz.Rect(rect),stream=data.getvalue(),keep_proportion=True)

def save(doc,path):
    doc.subset_fonts()
    doc.save(path,garbage=4,deflate=True)

def main():
    for sub in ['figures','overviews']:(ROOT/sub).mkdir(exist_ok=True)
    manifest={p['id']:p for p in json.loads((ROOT/'papers.json').read_text())}
    verified={}
    for name,meta in PAPERS.items():
        path=ROOT/'papers'/f'{name}.pdf';doc=fitz.open(path)
        p=dict(manifest[name])
        stamp=re.search(r'arXiv:\s*(\d+\.\d+v\d+)',doc[0].get_text())
        if stamp:p['url']='https://arxiv.org/pdf/'+stamp.group(1)
        p.update(task=meta[0],habitat=meta[1],evidence=meta[2],evidence_url=meta[3],pages=len(doc),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),local_pdf=f'papers/{name}.pdf',version=stamp.group(1) if stamp else '下载的正式版 / 来源 PDF')
        verified[name]=p
    entries=[]
    docs={n:fitz.open(ROOT/'papers'/f'{n}.pdf') for n in PAPERS}
    for i,row in enumerate(RAW,1):
        paper,figure,page,clip,group,priority,title,borrow,limit=row
        label=f'{i:02d}';slug=f'{label}_{paper}_Fig{figure}'.replace('–','-')
        src=docs[paper];rect=fitz.Rect(clip)
        assert src[page-1].rect.contains(rect),(paper,page,clip)
        part=fitz.open();target=part.new_page(width=rect.width,height=rect.height)
        target.show_pdf_page(target.rect,src,page-1,clip=rect)
        part.save(ROOT/'figures'/f'{slug}.pdf',garbage=4,deflate=True)
        src[page-1].get_pixmap(matrix=fitz.Matrix(3,3),clip=rect,alpha=False).save(ROOT/'figures'/f'{slug}.png')
        v=verified[paper]
        entries.append(dict(id=label,paper=paper,figure='Fig. '+figure,page=page,clip=clip,group=group,group_title=GROUPS[group],priority=priority,title=title,borrow=borrow,limitation=limit,task=v['task'],habitat=v['habitat'],source_url=v['url']+f'#page={page}',source_sha256=v['sha256'],png=f'figures/{slug}.png',pdf=f'figures/{slug}.pdf',source_pdf=v['local_pdf'],checked='已查看原 PDF 页和裁剪预览',excerpt=('子图' if figure[-1:].isalpha() else '相邻两图' if '–' in figure else '整图区域')))
    (ROOT/'sources.json').write_text(json.dumps(dict(research_date='2026-10-24',paper_count=len(verified),reference_units=len(entries),papers=verified,figures=entries),ensure_ascii=False,indent=2))
    with (ROOT/'参考图索引.csv').open('w',encoding='utf-8-sig',newline='') as fp:
        cols=['id','paper','figure','page','group_title','priority','title','task','habitat','borrow','limitation','source_url','png','pdf']
        w=csv.DictWriter(fp,fieldnames=cols,extrasaction='ignore');w.writeheader();w.writerows(entries)

    pack=fitz.open()
    for e in entries:
        p=pack.new_page(width=1000,height=730)
        box(p,[35,21,970,60],f"{e['id']}  {e['paper']} · {e['figure']}  |  {e['title']}",19)
        box(p,[35,65,970,103],f"PDF 第 {e['page']} 页 · 优先级 {e['priority']} · {e['task']} · Habitat：{e['habitat']}",11,MUTED)
        p.draw_rect(fitz.Rect(35,108,965,529),color=(.88,.90,.92),fill=(1,1,1))
        photo(p,[43,116,957,521],ROOT/e['png'])
        box(p,[35,543,965,598],'可借鉴：'+e['borrow'],13)
        box(p,[35,602,965,663],'与 Fig.8 的区别：'+e['limitation'],12,MUTED)
        source=f"来源：{e['source_url']}"
        box(p,[35,674,965,702],source,9,ACCENT)
        p.insert_link({'kind':fitz.LINK_URI,'from':fitz.Rect(35,674,965,702),'uri':e['source_url']})
        box(p,[35,706,965,726],f"2026-10-24 · {GROUPS[e['group']]} · {e['excerpt']} · 原图版权归论文作者 / 出版方",8,MUTED)
    pack.set_toc([[1,f"{e['id']} {e['paper']} {e['figure']} — {e['title']}",i+1] for i,e in enumerate(entries)])
    save(pack,ROOT/f"Fig8_完整参考图集_{len(entries)}项.pdf")

    # Fixed selection so the most relevant mechanism images are visible first.
    top_ids=['24','27','39','54','01','04','18','40','30','33','42','46']
    tops=[next(e for e in entries if e['id']==k) for k in top_ids]
    shortlist=fitz.open()
    for e in tops:
        i=entries.index(e)
        shortlist.insert_pdf(pack,from_page=i,to_page=i)
    shortlist.set_toc([[1,f"{e['id']} {e['paper']} {e['figure']}",i+1] for i,e in enumerate(tops)])
    save(shortlist,ROOT/'Fig8_优先12项详解.pdf')
    overview=fitz.open()
    chunks=[('最值得先看的 12 项',tops)]+[(GROUPS[g],[e for e in entries if e['group']==g]) for g in GROUPS]
    for section,arr in chunks:
        for start in range(0,len(arr),12):
            group=arr[start:start+12];p=overview.new_page(width=1200,height=1260)
            box(p,[28,20,1170,58],'Fig.8 参考图 · '+section,23)
            box(p,[28,64,1170,93],'编号对应完整图集和 CSV；点击缩略图可打开原论文。图号与页码以 sources.json 固定版本为准。',10,MUTED)
            for k,e in enumerate(group):
                x=28+(k%3)*392;y=103+(k//3)*281
                p.draw_rect(fitz.Rect(x,y,x+376,y+268),color=(.85,.88,.90),fill=(1,1,1))
                photo(p,[x+8,y+8,x+368,y+187],ROOT/e['png'],max_pixels=950)
                box(p,[x+9,y+194,x+369,y+220],f"{e['id']}  {e['paper']} · {e['figure']} · p{e['page']}",12)
                box(p,[x+9,y+222,x+369,y+263],e['title'],11,MUTED)
                p.insert_link({'kind':fitz.LINK_URI,'from':fitz.Rect(x,y,x+376,y+268),'uri':e['source_url']})
            filename=('00_优先看这12项' if section==chunks[0][0] else f'{len(overview)-1:02d}_分组总览')+'.png'
            p.get_pixmap(matrix=fitz.Matrix(1.35,1.35),alpha=False).save(ROOT/'overviews'/filename)
    save(overview,ROOT/'Fig8_分类速览.pdf')

    lines=[
        '# Fig.8 参考图扩展调研（2026-10-24）','',
        f'已下载并查看 **{len(verified)} 篇论文**，选出 **{len(entries)} 个参考单元**。一个参考单元可能是整图、明确标出的子图，或相邻两幅图；不将其混称为独立论文数量。',
        '这是针对你当前 Fig.8 的定向图像调研，不是 VLN 全领域的穷尽综述。既有 6 图参考包保留，本包复用其中有用来源并显著扩充。','',
        '**先打开：** [优先 12 项总览](overviews/00_优先看这12项.png) · [优先 12 项详解](Fig8_优先12项详解.pdf) · [分类速览 PDF](Fig8_分类速览.pdf) · '+f'[完整带说明图集](Fig8_完整参考图集_{len(entries)}项.pdf)'+' · [可筛选图库](index.html) · [CSV 索引](参考图索引.csv)','',
        '每个条目有：实际原图裁剪、论文固定版本、图号、PDF 页码、任务类别、Habitat 核实状态、可以借鉴的设计、与你机制的区别。`figures/` 同时提供 PNG 和保留原图矢量内容的 PDF 裁剪；`papers/` 是原论文。','',
        '## 与你的 Fig.8 对齐','',
        '依据 `PRE_MAP_VLN/论文/TASE_英文初稿/root.tex` 的 Fig.8 图注及执行章节，你需要讲清：未检出后允许的局部重试 → 局部候选耗尽/达到限制 → 同一目标换区域恢复 → 执行中成功验证 → 执行后将确认的新实例插入场景图。重复覆盖不能重复产生负证据；没有找到目标不能否定作为锚点的其他物体。','',
        '我最推荐 **BSC-Nav Fig.3b / SG-Nav Fig.6 的因果叙事 + BeliefMapNav Fig.12 的空间对照 + DovSG Fig.3 的局部更新**。前三篇有 Habitat 实验；DovSG 的所选图来自真实场景，仅补充地图更新画法。这是绘图取舍判断，不表示它们实现了你的方法。','',
        '## 优先选择','',
        '|编号|论文与图号|PDF 页|最适合借鉴|', '|---|---|---:|---|',
    ]
    for e in tops:lines.append(f"|{e['id']}|[{e['paper']} {e['figure']}]({e['png']})|{e['page']}|{e['title']}|")
    lines += ['', '## 四条调研线与全部原图','']
    for g,gn in GROUPS.items():
        lines += [f'### {g} · {gn}','','|编号|论文 / 图号|PDF 页|优先级|适合参考的内容|','|---|---|---:|---:|---|']
        for e in entries:
            if e['group']==g:lines.append(f"|{e['id']}|[{e['paper']} {e['figure']}]({e['png']}) · [原文]({e['source_url']})|{e['page']}|{e['priority']}|{e['title']}|")
        lines.append('')
    lines += ['优先级 1：先看；2：针对局部表达补充；3：备选或构图较复杂。排序依据是与你 Fig.8 的适配程度，不是论文质量。','', '## 仿真器与任务边界','',
              '“用了 HM3D / MP3D 数据”不自动等于“用了 Habitat 仿真器”。同一论文也可能同时包含 Habitat、其他仿真器和真实机器人图。本表在无法核实处保留不确定性。','',
              '|论文|任务|Habitat 状态|核实依据|','|---|---|---|---|']
    for n,v in verified.items():
        url=v['evidence_url'] or v['url'];lines.append(f"|[{n}]({v['url']})|{v['task']}|{v['habitat']}|[{v['evidence']}]({url})|")
    lines += ['', '## 按 Fig.8 的四个部分选择','',
        '- **局部重试：** SG-Nav 27、ETPNav 02、Regretful Agent 19。至少保留两次不同观测与触发原因；原图中的误检、避障、路线错误需转换成你的真实情形。',
        '- **新增覆盖与负证据：** VLFM 31 看重叠视野，BeliefMapNav 39 看空间信念对照，SEEK 50–51 看房间搜索。用固定视角标“已覆盖 / 本次新增”，不能只换热图颜色。',
        '- **跨区域恢复：** BSC-Nav 24、ETPNav 01、Regretful Agent 18、HOV-SG 46。主图应让读者看见起始区域、替代区域和连接路径。',
        '- **验证与地图更新：** BSC-Nav 24、UniGoal 40、ConceptGraphs 52–53、DovSG 54。成功验证和新实例插图分开标时点；Before / After 旧节点保持原位置，新增节点和边再高亮。','',
        '## 建议保留的三种构图方案','',
        '1. **主地图居中，关键观测环绕。** 参考 BSC-Nav 24、Regretful Agent 18、BeliefMapNav 33。最适合强调“同一目标换到哪里继续找”，右下角留小型地图更新对照。',
        '2. **三列时间序列，每列地图与 RGB 对齐。** 参考 CA-Nav 04、ETPNav 01、UniGoal 40。依次呈现局部重试、跨区域搜索、验证成功；更新前后图放在末列下方。',
        '3. **一张大空间主图 + 两个机制局部放大。** 参考 VLFM 30、HOV-SG 46、BeliefMapNav 39、DovSG 54。左侧放覆盖去重，右侧放实例更新；更适合你希望 Fig.8 有明确空间结构的情况。','',
        '不建议把完整系统框图、长段 LLM 对话、多个互不对应的三维小场景组合成主图。它们很难同时说明局部失败、跨区域路线和新实例的变化。','',
        '## 版本、来源与复核','',
        '- 每张裁剪都来自本地保存的原论文，未做生成式补图或改动原图数据。子图与相邻两图均有标记。',
        '- `sources.json` 记录 URL、页码、裁剪坐标、版本及 SHA-256。页码一律为 PDF 文件从 1 开始的页序，不是期刊印刷页码。',
        '- 本包下载到 CA-Nav v4、MapNav v5、ETPNav v3；BeliefMapNav / SG-Nav 保留此前参考包的 v1，因此不能拿其他版本图号直接对应。',
        '- BSC-Nav 使用 Nature Communications 2026 正式版；Fig.3b 与 Fig.4c 是不同任务的子图。',
        '- AECNav（arXiv:2608.10817）检索到但 PDF 本轮下载超时，未查看实际配图，因此不计入 27 篇和参考图数量。',
        '- 图像版权归原作者 / 出版方。本包用于来源明确的研究与构图参考；本项目论文与已有 Fig.8 文件没有改动。','',
        '复现：在本目录运行 `python build_pack.py`。下载器为 `download_papers.py`；它只读取明确列出的公开论文地址。','',
        '## 每项的具体借鉴与限制','',
    ]
    for e in entries:
        lines += [f"### {e['id']} · {e['paper']} {e['figure']} — {e['title']}",'',f"[原图 PNG]({e['png']}) · [裁剪 PDF]({e['pdf']}) · [论文第 {e['page']} 页]({e['source_url']})",'',e['borrow'],'','**区别：** '+e['limitation'],'']
    (ROOT/'README.md').write_text('\n'.join(lines),encoding='utf-8')

    # A self-contained offline index; original PDFs and images remain local.
    cards=[]
    for e in entries:
        esc=html.escape
        cards.append(f'''<article data-group="{e['group']}" data-priority="{e['priority']}" data-search="{esc(' '.join([e['paper'],e['title'],e['task'],e['habitat']]))}"><a href="{e['png']}" target="_blank"><img loading="lazy" src="{e['png']}" alt="{esc(e['paper']+' '+e['figure'])}"></a><h3>{e['id']} · {esc(e['paper'])} {e['figure']}</h3><p class="muted">PDF p{e['page']} · 优先级 {e['priority']} · {esc(e['task'])}<br>Habitat：{esc(e['habitat'])}</p><b>{esc(e['title'])}</b><p>{esc(e['borrow'])}</p><p class="muted">区别：{esc(e['limitation'])}</p><p><a href="{e['source_url']}" target="_blank">原论文</a> · <a href="{e['pdf']}">裁剪 PDF</a> · <a href="{e['source_pdf']}#page={e['page']}">本地论文</a></p></article>''')
    options=''.join(f'<option value="{g}">{g} · {html.escape(n)}</option>' for g,n in GROUPS.items())
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Fig.8 论文参考图库</title><style>body{{font-family:system-ui,sans-serif;background:#f3f5f6;color:#22303d;margin:0;padding:32px}}header{{max-width:1100px}}h1{{font-size:28px}}a{{color:#116b79}}.controls{{position:sticky;top:0;background:#f3f5f6;padding:16px 0;display:flex;gap:12px;flex-wrap:wrap;z-index:2}}input,select{{padding:10px;border:1px solid #bccbd1;border-radius:6px;font-size:14px}}main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:20px}}article{{background:white;padding:18px;border:1px solid #dce3e6;border-radius:8px}}article img{{width:100%;height:230px;object-fit:contain}}h3{{margin:14px 0 8px}}p{{line-height:1.6;font-size:14px}}.muted{{color:#5b6874}}article[hidden]{{display:none}}</style><header><h1>Fig.8 论文参考图库</h1><p>{len(verified)} 篇论文 · {len(entries)} 个参考单元 · 2026-10-24。点击图片放大。每项区分原任务、Habitat 使用情况及与你机制的差异。</p><p><a href="README.md">完整调研说明</a> · <a href="Fig8_分类速览.pdf">分类速览</a> · <a href="Fig8_完整参考图集_{len(entries)}项.pdf">完整图集</a></p></header><div class="controls"><input id="q" placeholder="检索论文、目标、信念、VLN…"><select id="g"><option value="">全部主题</option>{options}</select><select id="p"><option value="">全部优先级</option><option value="1">优先级 1</option><option value="2">优先级 2</option><option value="3">优先级 3</option></select><span id="count"></span></div><main>{''.join(cards)}</main><script>const q=document.querySelector('#q'),g=document.querySelector('#g'),p=document.querySelector('#p');function filter(){{let n=0;document.querySelectorAll('article').forEach(a=>{{const show=(!g.value||a.dataset.group===g.value)&&(!p.value||a.dataset.priority===p.value)&&a.dataset.search.toLowerCase().includes(q.value.toLowerCase());a.hidden=!show;n+=show;}});document.querySelector('#count').textContent=n+' 项';}}[q,g,p].forEach(e=>e.addEventListener('input',filter));filter();</script></html>'''
    (ROOT/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps(dict(papers=len(verified),reference_units=len(entries),pdf_pages=len(pack),overview_pages=len(overview),top12=[f"{e['id']} {e['paper']} {e['figure']}" for e in tops]),ensure_ascii=False))

if __name__=='__main__':main()
