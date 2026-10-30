"""Build a visual study atlas from actual paper figures and project images."""
from pathlib import Path
import json
import subprocess
import concurrent.futures
import xml.etree.ElementTree as ET
from PIL import Image,ImageDraw,ImageFont
import numpy as np

HERE=Path(__file__).resolve().parent
PAPER=HERE.parents[2]
BASE=PAPER/'references/figure_examples'
OLD=BASE/'主图参考_20261029'
TMP=Path('/tmp/logicfly_teaser_highres_pages');TMP.mkdir(exist_ok=True)
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
records=[]


def local(name,source,box,page,notes,url):
    records.append({'name':name,'role':f'Fig. 1 · PDF p.{page}','pdf':str(source),'page':page,
                    'box_fraction':box,'notes':notes,'url':url,'kind':'paper_figure'})


def remote(name,image,notes,url,role='作者项目页展示图'):
    records.append({'name':name,'role':role,'image':str(image),'notes':notes,'url':url,'kind':'project_image'})


local('FALCON',BASE/'候选视角生成/papers/FALCON.pdf',(.07,.17,.96,.40),1,
      '最接近本任务：长条三维总轨迹、下排实拍、右侧局部放大整合为一个紧凑矩形。可借鉴实测体素几何的清晰轮廓；不是把稀疏点云简单染灰。',
      'https://arxiv.org/abs/2407.00577')
remote('USS-Nav',BASE/'d4e448ba664fe6f2e2ab415d11f14d8f.jpg',
       '已核对是你原参考目录中的那张图。竖排观察图紧靠三维主场景，信息密度高。借鉴场景与观察列的比例；硬件资源统计、房间分区标签不属于本稿主图重点。',
       'https://arxiv.org/abs/2602.00708','Fig. 1 · 已与下载的论文 PDF 核对')
local('HOV-SG',BASE/'语义恢复参考/扩展调研_20261024/papers/HOV-SG.pdf',(.50,.25,.96,.495),1,
      '三维场景占主位，观察图贴近相关区域，连接短而明确。可借鉴观察图与空间位置的关系；本实验为单层场景，不照搬其楼层展开和房间命名。',
      'https://hovsg.github.io/')
remote('ActiveSplat',HERE/'ActiveSplat_img1.png',
       '空间质感参考：完整的墙、地面和物体表面让轨迹有依托。作者此处展示数据集上的映射与重建；不应当作我们的真机照片，也不能用配色冒充同等重建质量。',
       'https://li-yuetao.github.io/ActiveSplat/','作者项目页 teaser · 数据集实验呈现')
remote('See, Point, Fly',OLD/'SPF_image_3.png',
       '局部场景、连续轨迹和短指令构成独立案例，空间范围经过选择。可借鉴局部放大和相机位姿表达；不必把整条空走廊当成最大画面。',
       'https://arxiv.org/html/2509.22653v1','Fig. 1')
local('ConceptGraphs',BASE/'候选视角生成/papers/ConceptGraphs.pdf',(.07,.23,.96,.525),1,
      '完整的剖开式三维场景、少量语义突出和任务示例形成层次。可借鉴场景切面与主次关系；不直接照搬彩色语义分区和多种任务模块。',
      'https://concept-graphs.github.io/')
local('VLMaps',BASE/'候选视角生成/papers/VLMaps.pdf',(.51,.15,.96,.37),1,
      '场景、机器人和语言约束在一个主要视图中读完，路径依附于可理解的物体。提醒我们：清楚的真实场景关系比增加图例和标注更重要。',
      'https://vlmaps.github.io/')
local('VLFM',BASE/'语义恢复参考/前期参考调研_20261024/papers/VLFM.pdf',(.07,.18,.96,.48),1,
      '在有纹理的场景上直接读出目标、起点和路线差异，外围说明克制。可借鉴路线可读性；不是本任务的三维高度和观察位姿展示模板。',
      'https://naoki.io/portfolio/vlfm')
remote('IGL-Nav',HERE/'IGL_Nav_img1.png',
       '大幅实拍、跨时刻机器人叠加和下方渲染序列表达运动。多时刻实例需要一致的画面坐标；当前移动跟拍不能未经配准直接套用这种叠加。',
       'https://gwxuan.github.io/IGL-Nav/')
local('FUEL',BASE/'候选视角生成/papers/FUEL.pdf',(.54,.20,.96,.56),1,
      '真实实验照片与在线地图相互验证。可借鉴实物与地图配对；高度或占用着色有其含义，不为了视觉丰富随意增加彩虹色。',
      'https://arxiv.org/abs/2010.11561')
remote('GRaD-Nav++',OLD/'GRaD_Nav_project_image_2.png',
       '实景背景中的轨迹具有直观空间关系。可借鉴动作表达；轨迹、机器人实例与背景的一致性需要真实位姿和投影，不用手绘线替代记录。',
       'https://qianzhong-chen.github.io/gradnavpp.github.io/','作者项目页长期任务示例')
remote('OnFly',OLD/'OnFly_image_3.png',
       '实拍场景是视觉主体，平台、机载图和重建形成不同层次。当前作者偏好白底三维图，因此主要学习主次关系，不继续照搬照片混合背景。',
       'https://arxiv.org/html/2603.10682v1','Fig. 1')
remote('AirHunt',OLD/'AirHunt_image_3.png',
       '大幅鸟瞰实拍承载完整路线，硬件和机载图居次。当前没有这种覆盖全路线的外部视角；不把它当成仅靠换版式就能复刻的方案。',
       'https://arxiv.org/html/2601.12742v1','Fig. 1')
local('InstructNav',BASE/'语义恢复参考/扩展调研_20261024/papers/InstructNav.pdf',(.17,.20,.78,.535),2,
      '用多种真实任务图证明泛化能力，短指令紧跟照片。适合能力集锦；本图是一条连续多任务飞行，不能拆成互不关联的照片展板。',
      'https://arxiv.org/abs/2406.04882')
local('VoxPoser',BASE/'候选视角生成/papers/VoxPoser.pdf',(.10,.575,.96,.79),1,
      '少量机制示意配合密集、统一裁剪的真实任务图，形成稳定网格。可借鉴图像尺度统一；不让辅助图与三维主图争夺同等权重。',
      'https://proceedings.mlr.press/v229/huang23b.html')
local('SG-Nav',BASE/'语义恢复参考/前期参考调研_20261024/papers/SG-Nav.pdf',(.17,.09,.80,.30),2,
      '用三维室内模型和相反的路线行为解释方法差异，叙事明确。可借鉴“一个空间事件讲清一个贡献”；不复制任务无关的图结构、对话和房间标签。',
      'https://arxiv.org/abs/2410.08189')
local('DovSG',BASE/'语义恢复参考/前期参考调研_20261024/papers/DovSG.pdf',(.07,.14,.96,.49),1,
      '系统总览通过感知、记忆、规划和执行模块串联真实任务。它适合方法总图；不作为当前真机主图骨架，避免回到汇报式模块堆叠。',
      'https://arxiv.org/abs/2410.11989')
local('OpenFMNav',BASE/'语义恢复参考/早期补充原图/OpenFMNav.pdf',(.51,.20,.96,.425),1,
      '以语言输入和成功/失败对照说明问题设置。可以学习简洁地交代输入；对本图的真机轨迹质感帮助有限，不作为主要视觉参考。',
      'https://arxiv.org/abs/2402.10670')
remote('Splat-Nav',HERE/'Splat_Nav_img1.png',
       '这是系统流程型 Fig. 1，而非纯实验主图。可借鉴三维场景与定位/规划的联系；不把复杂工作流带进当前真机执行主图。',
       'https://arxiv.org/html/2403.02751v3','Fig. 1 · 系统流程图')
remote('RT-GuIDE',HERE/'RT_GuIDE_img1.png',
       '作者项目页的系统图把重建、拓扑和轨迹放在同一链条。适合方法图参考；当前不是要重画系统流程，因此不直接套用模块框。',
       'https://tyuezhan.github.io/RT_GuIDE/','作者项目页系统图 · 非 Fig. 1 裁图')
local('BSC-Nav',BASE/'语义恢复参考/扩展调研_20261024/papers/BSC-Nav.pdf',(.065,.38,.95,.84),2,
      '图形、字体和层级经过统一，但内容是认知机制与系统框架。可借鉴标注密度和字体一致性，不照搬与真机轨迹无关的流程模块。',
      'https://www.nature.com/articles/s41467-026-74358-5')


def prepare(record):
    if 'pdf' not in record:
        record['figure_image']=record['image'];return record
    name=record['name'].replace(',','').replace(' ','_')
    image_path=TMP/(name+f"_p{record['page']}.png")
    if not image_path.exists():
        subprocess.run(['pdftoppm','-f',str(record['page']),'-singlefile','-r','240','-png',record['pdf'],str(image_path.with_suffix(''))],check=True,capture_output=True)
    im=Image.open(image_path).convert('RGB');w,h=im.size
    x0,y0,x1,y1=record['box_fraction']
    # Use the actual caption position as the lower edge, so photo strips are
    # retained in full even when they extend beyond an approximate initial box.
    xml=subprocess.run(['pdftotext','-f',str(record['page']),'-l',str(record['page']),'-bbox-layout',record['pdf'],'-'],
                       check=True,capture_output=True,text=True).stdout
    namespace={'x':'http://www.w3.org/1999/xhtml'}
    document=ET.fromstring(xml)
    pdf_page=document.find('.//x:page',namespace)
    caption_edges=[]
    for row in document.findall('.//x:line',namespace):
        words=row.findall('x:word',namespace)
        if len(words)>1 and words[0].text in ['Fig.','Figure'] and (words[1].text or '').startswith('1'):
            caption_edges.append(float(row.attrib['yMin'])/float(pdf_page.attrib['height']))
    if caption_edges:y1=min(caption_edges)-.003
    crop=(round(x0*w),round(y0*h),round(x1*w),round(y1*h))
    im=im.crop(crop)
    # Remove author text/whitespace above the colored figure while retaining the
    # manually selected horizontal extent and the whole chosen figure region.
    a=np.asarray(im);mask=(a.max(2)-a.min(2)>30)&(a.min(2)<235)
    ys=np.where(mask)[0]
    if len(ys) and record['name']!='FALCON':
        top=max(0,int(ys.min())-20)
        im=im.crop((0,top,im.width,im.height))
    dest=HERE/(name+'_figure.png');im.save(dest)
    record['figure_image']=str(dest);record['render_dpi']=240;record['initial_pixel_crop']=crop
    return record


with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    records=list(pool.map(prepare,records))
(HERE/'figure_study_index.json').write_text(json.dumps(records,indent=2,ensure_ascii=False))

def font(size,bold=False):return ImageFont.truetype(BOLD if bold else FONT,size)


def wrapped(draw,xy,content,face,max_width,fill,spacing=12):
    x,y=xy;line=''
    for char in content:
        if char=='\n' or (line and draw.textlength(line+char,font=face)>max_width):
            draw.text((x,y),line,font=face,fill=fill);y+=face.size+spacing;line=''
            if char=='\n':continue
        line+=char
    if line:draw.text((x,y),line,font=face,fill=fill)
    return y+face.size


pages=[]
for first in range(0,len(records),3):
    page=Image.new('RGB',(2400,3300),'white');draw=ImageDraw.Draw(page)
    draw.text((80,35),'论文主图视觉研究 · 2026-10-30',font=font(42,True),fill='#223341')
    draw.text((2320,40),f'{first//3+1} / 7',font=font(34),fill='#7B8791',anchor='ra')
    for row,r in enumerate(records[first:first+3]):
        y=130+row*1030
        draw.text((80,y),f"{first+row+1:02d}  {r['name']}",font=font(50,True),fill='#1D303E')
        draw.text((2320,y+10),r['role'],font=font(30),fill='#6E7B85',anchor='ra')
        a=Image.open(r['figure_image']).convert('RGB')
        scale=min(2240/a.width,665/a.height)
        a=a.resize((round(a.width*scale),round(a.height*scale)),Image.Resampling.LANCZOS)
        page.paste(a,((2400-a.width)//2,y+80+(665-a.height)//2))
        wrapped(draw,(80,y+765),r['notes'],font(35),2240,'#33454F',10)
        draw.text((80,y+965),r['url'],font=font(25),fill='#55718A')
        if row<2:draw.line((80,y+1000,2320,y+1000),fill='#DDE4E9',width=2)
    pages.append(page)
    page.resize((1200,1650),Image.Resampling.LANCZOS).save(HERE/f'atlas_page_{first//3+1}.jpg',quality=95)
pages[0].save(HERE/'主图参考研究_21例.pdf',save_all=True,append_images=pages[1:],resolution=280,quality=92)

# A focused, readable board of the four most useful references.
selection=[records[i] for i in [0,1,2,3]]
board=Image.new('RGB',(2400,1830),'white');draw=ImageDraw.Draw(board)
draw.text((50,20),'重点参考：构图、信息密度与三维场景质感',font=font(46,True),fill='#233642')
short_notes={
    'FALCON':'长场景总轨迹＋局部放大，组合紧凑。',
    'USS-Nav':'竖排观察紧贴三维主场景。',
    'HOV-SG':'观察图靠近对应位置，主次明确。',
    'ActiveSplat':'完整表面与剖开视图；仅作场景质感参考。',
}
for i,r in enumerate(selection):
    x=50+i%2*1200;y=110+i//2*850
    draw.text((x,y),r['name'],font=font(42,True),fill='#1F3342')
    a=Image.open(r['figure_image']).convert('RGB')
    scale=min(1100/a.width,620/a.height)
    a=a.resize((round(a.width*scale),round(a.height*scale)),Image.Resampling.LANCZOS)
    board.paste(a,(x+(1100-a.width)//2,y+65+(620-a.height)//2))
    wrapped(draw,(x,y+715),short_notes[r['name']],font(32),1100,'#526570',8)
board.save(HERE/'重点主图对照.jpg',quality=97)
board.save(HERE/'重点主图对照.pdf',resolution=220)
print('Built atlas: 21 paper/project examples, including 17 newly inspected papers; 7 pages.')
