"""Box-prompted SAM2 adapter using the official Transformers interface.

See https://huggingface.co/docs/transformers/model_doc/sam2 . Returns one
boolean mask per proposal at the original RGB resolution; no model resizing
coordinates are passed downstream as original-image coordinates.
"""
import os
import numpy as np
import torch
from transformers import Sam2Model, Sam2Processor


class Sam2BoxSegmenter:
    def __init__(self, device='cuda', model_name=None, mask_threshold=0.):
        self.device = torch.device(device)
        self.model_name = model_name or os.environ.get('PRE_MAP_VLN_SAM2_MODEL', 'facebook/sam2.1-hiera-large')
        self.processor = Sam2Processor.from_pretrained(self.model_name)
        self.model = Sam2Model.from_pretrained(self.model_name).to(self.device).eval()
        self.mask_threshold = float(mask_threshold)

    @torch.inference_mode()
    def segment(self, image, boxes_xyxy):
        if not len(boxes_xyxy):
            return np.empty((0, image.height, image.width), dtype=bool)
        inputs = self.processor(images=image, input_boxes=[boxes_xyxy], return_tensors='pt').to(self.device)
        outputs = self.model(**inputs, multimask_output=False)
        masks = self.processor.post_process_masks(
            outputs.pred_masks.cpu(), inputs['original_sizes'].cpu(),
            mask_threshold=self.mask_threshold, binarize=True,
        )[0]
        masks = masks[:, 0].numpy().astype(bool)
        if masks.shape != (len(boxes_xyxy), image.height, image.width):
            raise RuntimeError('SAM2 mask shape does not match original RGB image')
        return masks
