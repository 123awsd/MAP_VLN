from pathlib import Path
import json,numpy as np,torch
from PIL import Image
from transformers import AutoImageProcessor,SegformerForSemanticSegmentation
root=Path(__file__).resolve().parents[1]/'outputs/semantic_structure_trial';data=root/'rgbd';out=root/'semantics';out.mkdir(exist_ok=True)
modeldir=next((root/'models/hub/models--nvidia--segformer-b2-finetuned-ade-512-512/snapshots').iterdir())
processor=AutoImageProcessor.from_pretrained(modeldir);model=SegformerForSemanticSegmentation.from_pretrained(modeldir).eval().to('cuda');torch.set_num_threads(4)
(root/'labels.json').write_text(json.dumps(model.config.id2label,ensure_ascii=False,indent=2))
for i,path in enumerate(sorted(data.glob('*.png'))):
 image=Image.open(path).convert('RGB');inputs=processor(images=image,return_tensors='pt').to('cuda')
 with torch.inference_mode():
  logits=model(**inputs).logits;logits=torch.nn.functional.interpolate(logits,size=(image.height,image.width),mode='bilinear',align_corners=False);prob=logits.softmax(1);conf,labels=prob.max(1)
 lab=labels[0].cpu().numpy().astype('uint8');confidence=conf[0].cpu().numpy().astype('float16');np.savez_compressed(out/(path.stem+'.npz'),labels=lab,confidence=confidence)
 if i%10==0:
  rgb=np.array(image);palette=np.array([[220,220,220],[100,160,200],[180,130,110],[110,170,130]],dtype='uint8');color=np.zeros_like(rgb);color[lab==0]=palette[0];color[lab==3]=palette[1];color[lab==5]=palette[2];color[lab==14]=palette[3];overlay=(rgb*.5+color*.5).astype('uint8');Image.fromarray(np.concatenate([rgb,overlay],axis=1)).save(out/(path.stem+'_检查.png'))
 print(i+1,path.stem,flush=True)
