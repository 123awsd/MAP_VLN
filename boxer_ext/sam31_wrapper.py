"""Local SAM3.1 detector adapter for Boxer's (xmin,xmax,ymin,ymax) contract.

Uses the official multiplex detector assembly, strictly loads its weights,
then runs image inference only. Video tracking is deliberately not implied.
"""
from pathlib import Path
import sys
import numpy as np
import torch
from PIL import Image
from torchvision.ops import batched_nms

ROOT = Path(__file__).resolve().parents[1]


class Sam31Wrapper:
    def __init__(self, device='cuda', text_prompts=None, min_confidence=.5,
                 checkpoint_path=None, precision=None):
        sys.path.insert(0, str(ROOT/'third_party/sam3_source'))
        from safetensors.torch import load_file
        from sam3 import model_builder as mb
        from sam3.model.sam3_multiplex_detector import Sam3MultiplexDetector
        from sam3.model.vl_combiner import SAM3VLBackboneTri
        from sam3.model.sam3_image_processor import Sam3Processor
        self.device = str(device)
        self.prompts = [str(v).strip().lower() for v in (text_prompts or [])]
        if not self.prompts or len(set(self.prompts)) != len(self.prompts):
            raise ValueError('SAM3.1 requires nonempty, unique text prompts')
        self.checkpoint_path = Path(checkpoint_path or ROOT/'models/sam31/sam3.1_multiplex.safetensors').resolve()
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(self.checkpoint_path)
        self.precision = precision
        visual = mb._create_multiplex_tri_backbone(compile_mode=None, use_fa3=False, use_rope_real=False)
        bpe = str(ROOT/'third_party/sam3_source/sam3/assets/bpe_simple_vocab_16e6.txt.gz')
        backbone = SAM3VLBackboneTri(scalp=0, visual=visual, text=mb._create_text_encoder(bpe))
        model = Sam3MultiplexDetector(
            num_feature_levels=1, backbone=backbone,
            transformer=mb._create_sam3_transformer(use_fa3=False),
            segmentation_head=mb._create_segmentation_head(use_fa3=False),
            semantic_segmentation_head=None, input_geometry_encoder=mb._create_geometry_encoder(),
            use_early_fusion=True, use_dot_prod_scoring=True,
            dot_prod_scoring=mb._create_dot_product_scoring(),
            supervise_joint_box_scores=True, is_multiplex=True)
        weights = load_file(str(self.checkpoint_path), device='cpu')
        detector_weights = {k[len('detector.'):]:v for k,v in weights.items() if k.startswith('detector.')}
        model.load_state_dict(detector_weights, strict=True)
        del weights, detector_weights
        self.model = model.to(self.device).eval()
        self.processor = Sam3Processor(self.model, device=self.device, confidence_threshold=float(min_confidence))
        self.last_detections = []
        self.last_unmapped_detections = []

    @torch.inference_mode()
    def forward(self, image_torch, resize_to_HW=None):
        if image_torch.ndim != 4 or image_torch.shape[0] != 1:
            raise ValueError('expected image [1,3,H,W], values 0..255')
        image = Image.fromarray(image_torch[0].clamp(0,255).byte().permute(1,2,0).cpu().numpy())
        boxes, scores, ids, masks = [], [], [], []
        enabled = self.device.startswith('cuda') and self.precision != 'float32'
        with torch.autocast('cuda', dtype=torch.bfloat16, enabled=enabled):
            state = self.processor.set_image(image)
            for index, prompt in enumerate(self.prompts):
                result = self.processor.set_text_prompt(prompt, state)
                n = len(result['scores'])
                if n:
                    boxes.append(result['boxes'].float().cpu())
                    scores.append(result['scores'].float().cpu())
                    ids.extend([index]*n)
                    masks.append(result['masks'][:,0].cpu())
        if not boxes:
            self.last_detections = []
            return torch.empty((0,4)), torch.empty(0), torch.empty(0,dtype=torch.long), np.empty((0,image.height,image.width),bool)
        boxes, scores, ids, masks = torch.cat(boxes), torch.cat(scores), torch.tensor(ids), torch.cat(masks)
        keep = batched_nms(boxes, scores, ids, .5)
        cross = batched_nms(boxes[keep], scores[keep], torch.zeros(len(keep),dtype=torch.long), .9)
        keep = keep[cross]
        boxes, scores, ids, masks = boxes[keep], scores[keep], ids[keep], masks[keep]
        self.last_detections = [{'label':self.prompts[int(i)],'score':float(s),'bbox_xyxy':b.tolist()}
                                for b,s,i in zip(boxes,scores,ids)]
        return boxes[:,[0,2,1,3]], scores, ids, masks.numpy()
