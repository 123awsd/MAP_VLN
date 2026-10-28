"""Extract attributed paper figures and assemble a review-only reference pack.

Original figure contents and colors are preserved. The selected lower example
of SG-Nav Fig. 6 is explicitly marked as an excerpt. No Fig. 8 assets are edited.
"""
from pathlib import Path
import hashlib
import json
import fitz

ROOT = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
INK = (0.12, 0.15, 0.19)
MUTED = (0.36, 0.39, 0.43)
ENTRIES = [
    dict(id='01', paper='VLFM', title='真实场景 + 半透明信息层',
         source='VLFM.pdf', page=1, figure='Fig. 1', clip=[54, 155, 558, 336],
         url='https://arxiv.org/pdf/2312.03275v1#page=1',
         borrow='空间位置、搜索路径和数值层共用一张底图，读者无需另猜抽象符号。',
         fit='适合 Fig. 8 的主场景，以及“去哪里继续找”的空间关系。',
         limit='原图表达目标搜索的语义价值与路径比较，不直接表示覆盖驱动的负证据更新。',
         short='看空间构图与透明叠层；配色可以另定。'),
    dict(id='02', paper='BeliefMapNav', title='一张主地图串起多个观测时刻',
         source='BeliefMapNav.pdf', page=2, figure='Fig. 1', clip=[107, 58, 506, 259],
         url='https://arxiv.org/pdf/2506.06487v1#page=2',
         borrow='中间地图承担空间主线，上下观测对应地图上的时间节点，有明确主次。',
         fit='适合把局部重试、切换搜索区域和最终验证放在同一段执行过程里。',
         limit='原图的六个时间点、彩色体素和路径细节较多；小尺寸 Fig. 8 应只保留关键转折。',
         short='看主次布局；不必把所有子图等宽并排。'),
    dict(id='03', paper='SG-Nav', title='误检 → 再观察 → 确认目标',
         source='SG-Nav.pdf', page=9, figure='Fig. 6, lower example',
         clip=[108, 248, 504, 412],
         url='https://arxiv.org/pdf/2410.08189v1#page=9',
         borrow='连续相机观测和固定的局部地图，直接解释为何重新判断目标。',
         fit='适合解决“失败以后具体看到了什么、为什么继续找”的直观表达。',
         limit='这是原图下半部的单个例子。它纠正目标误检，与本论文的未检出恢复并非同一机制；借鉴因果呈现。',
         short='看失败与恢复的因果关系；此处截取下半部。'),
    dict(id='04', paper='DovSG', title='新观测对应地图的局部更新',
         source='DovSG_v6.pdf', page=5, figure='Fig. 3', clip=[311, 57, 564, 217],
         url='https://arxiv.org/pdf/2410.11989v6#page=5',
         borrow='用真实局部观测与更新前后地图的对应区域，交代“哪里发生了变化”。',
         fit='适合 Fig. 8 的地图更新部分；可把观测、确认实例及对应图节点建立明确关联。',
         limit='原图含重定位和配准流程，不能照搬为本论文的融合模块；本论文还需保留执行后插入实例的时间关系。',
         short='看局部变化如何落回地图；左侧算法细节无需照搬。'),
    dict(id='05', paper='BeliefMapNav', title='同一视角下比较三层空间证据',
         source='BeliefMapNav.pdf', page=20, figure='Fig. 12', clip=[116, 323, 496, 684],
         url='https://arxiv.org/pdf/2506.06487v1#page=20',
         borrow='三个视角完全一致的场景上下排列，便于对照先验、可见性与后验。',
         fit='适合参考“置信度变化仍然附着在具体空间上”的表达，避免孤立的小网格。',
         limit='原文是体素信念与可见性，本论文是区域信念与新增覆盖；不把三张全图直接塞入小尺寸 Fig. 8。',
         short='看固定视角的上下对照；与本论文更新公式不同。'),
    dict(id='06', paper='Inner Monologue', title='动作与反馈一一对应',
         source='InnerMonologue.pdf', page=2, figure='Fig. 1', clip=[107, 70, 505, 247],
         url='https://proceedings.mlr.press/v205/huang23c/huang23c.pdf#page=2',
         borrow='动作旁边就是成功或失败反馈，重试发生的原因沿阅读顺序展开。',
         fit='适合借鉴少量反馈文字的摆放与连接方式，是主场景以外的补充参考。',
         limit='空间信息较弱，若以对话气泡为主体仍会偏文字化；仅取反馈对应关系。',
         short='看反馈信息的摆放；作为局部表达参考。'),
]


def text(page, xy, value, size=12, color=INK, english=False):
    if not english:
        page.insert_font(fontname='CJK', fontfile=FONT)
    page.insert_text(xy, value, fontsize=size,
                     fontname='helv' if english else 'CJK', color=color)


def wrapped(page, rect, value, size=13, color=INK):
    page.insert_font(fontname='CJK', fontfile=FONT)
    result = page.insert_textbox(rect, value, fontname='CJK', fontsize=size,
                                 lineheight=1.5, color=color)
    if result < 0:
        raise ValueError(f'Text overflow: {value}')


def main():
    out = ROOT / 'figures'
    out.mkdir(exist_ok=True)
    pack = fitz.open()
    overview = pack.new_page(width=1200, height=1330)
    text(overview, (36, 43), 'Fig. 8 参考图候选', 24)
    text(overview, (36, 71), '六张论文原图节选｜供选择构图与表达方式｜2026-10-24', 13, MUTED)
    text(overview, (36, 94), '原图颜色保留；编号是候选编号，并非 Fig. 8 子图编号。', 12, MUTED)
    documents = {}
    for idx, entry in enumerate(ENTRIES):
        source = ROOT / 'papers' / entry['source']
        documents[entry['source']] = doc = documents.get(entry['source']) or fitz.open(source)
        clip = fitz.Rect(entry['clip'])
        stem = entry['id'] + '_' + entry['paper'].replace(' ', '_')
        extracted = fitz.open()
        p = extracted.new_page(width=clip.width, height=clip.height)
        p.show_pdf_page(p.rect, doc, entry['page'] - 1, clip=clip)
        extracted.save(out / (stem + '.pdf'), garbage=4, deflate=True)
        p.get_pixmap(matrix=fitz.Matrix(4, 4), alpha=False).save(out / (stem + '.png'))
        entry['preview'] = 'figures/' + stem + '.png'
        entry['crop_pdf'] = 'figures/' + stem + '.pdf'
        entry['source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()

        col, row = idx % 2, idx // 2
        x, y = 36 + col * 580, 123 + row * 391
        text(overview, (x, y + 22), entry['id'] + '  ' + entry['title'], 18)
        text(overview, (x, y + 46), entry['paper'] + ' | ' + entry['figure'] +
             ' | PDF p.' + str(entry['page']), 12, MUTED, english=True)
        overview.show_pdf_page(fitz.Rect(x, y + 61, x + 548, y + 320), extracted, 0)
        wrapped(overview, fitz.Rect(x, y + 334, x + 548, y + 374), entry['short'], 13, MUTED)
        overview.draw_line((x, y + 382), (x + 548, y + 382), color=(0.84, 0.86, 0.88), width=.6)

        detail = pack.new_page(width=1000, height=850)
        text(detail, (42, 45), entry['id'] + '  ' + entry['title'], 23)
        text(detail, (42, 72), entry['paper'] + ' | ' + entry['figure'] +
             ' | PDF p.' + str(entry['page']), 14, MUTED, english=True)
        detail.show_pdf_page(fitz.Rect(42, 98, 958, 570), extracted, 0)
        wrapped(detail, fitz.Rect(42, 594, 958, 641), '可借鉴：' + entry['borrow'], 15)
        wrapped(detail, fitz.Rect(42, 645, 958, 691), '与你的图：' + entry['fit'], 15)
        wrapped(detail, fitz.Rect(42, 695, 958, 753), '差异与取舍：' + entry['limit'], 13, MUTED)
        text(detail, (42, 791), entry['url'], 10, MUTED, english=True)
        detail.insert_link({'kind': fitz.LINK_URI, 'from': fitz.Rect(42, 776, 958, 800), 'uri': entry['url']})
        text(detail, (42, 823), '来源：各论文作者。此页为参考调研整理，原图未重绘、未改色。', 11, MUTED)
        extracted.close()
        # Adding a new page invalidates previously held Page instances.
        overview = pack[0]

    pack.subset_fonts()
    pack.save(ROOT / 'Fig8_reference_selection.pdf', garbage=4, deflate=True)
    pack[0].get_pixmap(matrix=fitz.Matrix(1.8, 1.8), alpha=False).save(ROOT / 'Fig8_reference_overview.png')
    (ROOT / 'sources.json').write_text(json.dumps(ENTRIES, ensure_ascii=False, indent=2) + '\n')
    lines = [
        '# Fig. 8 配图参考调研（2026-10-24）', '',
        '本轮是原图调研与选择，不是新一版 Fig. 8。未修改论文和此前绘图文件。', '',
        '先看 `Fig8_reference_overview.png`；`Fig8_reference_selection.pdf` 包含总览和六张带说明的大图。',
        '各原始图的独立 PNG / PDF 在 `figures/`，下载的论文在 `papers/`。', '',
        '检视了 VLFM、BeliefMapNav、SG-Nav、DovSG、Inner Monologue 和 SayPlan 的实际配图。',
        'SayPlan Fig. 1 适合理解系统流程，但模块与文字较多；Inner Monologue Fig. 5 同样偏文字，未列入候选。', '',
        '优先比较 01 和 02 的整体构图；03 和 04 用于判断是否能直观解释恢复与地图更新。05、06 是局部表达补充。',
        '这是一组视觉参考，不主张照搬任一论文的方法、数值或完整布局。', '',
    ]
    for e in ENTRIES:
        lines += [f"## {e['id']} — {e['title']}", '',
                  f"**{e['paper']}，{e['figure']}，PDF 第 {e['page']} 页。** [论文原文]({e['url']})", '',
                  f"![{e['paper']} {e['figure']}]({e['preview']})", '',
                  '可借鉴：' + e['borrow'], '', '与你的图：' + e['fit'], '',
                  '差异与取舍：' + e['limit'], '']
    lines += ['## 对此前 Fig. 8 的判断', '',
              'B、C 中的孤立网格、方块和缩小的三维视图未能对应同一具体场景，读者需要先解码符号，才能理解恢复过程。',
              '参考中的透明层有效，是因为它们承载数值或空间范围，而且能与底图中的实际位置对齐；透明材质本身不能解决叙事问题。',
              '下一轮应在选定整体构图后，围绕同一例子组织“未检出—局部重试—跨区域恢复—确认—执行后更新”。',
              '原文必须保留的区别：重复覆盖不重复增加负证据；锚点物体仍然有效；目标成功验证发生在执行中，确认新实例插图发生在执行后。', '',
              '来源与裁剪坐标、页码、PDF SHA-256 见 `sources.json`。所有原图版权归各论文作者。', '']
    (ROOT / 'README.md').write_text('\n'.join(lines))
    print('Created 7-page review PDF, overview PNG, six figure crops and source notes.')


if __name__ == '__main__':
    main()
