import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from classify_room_semantics_qwen import QwenRoomSemanticClassifier
from classify_coverage700_rooms_multiview import make_inputs


class MultiviewRoomSemanticTest(unittest.TestCase):
    def test_measured_views_and_boxer_are_same_auto_room(self):
        with tempfile.TemporaryDirectory() as tmp:
            rooms, images, records = make_inputs(Path(tmp))
            room=next(r for r in rooms if r['display_id']=='2-05')
            self.assertEqual(room['id'],205)
            self.assertEqual(len(images['205']),2)
            self.assertTrue(all(v['frame_boxer_detections']>0 for v in records['2-05']))
            self.assertTrue({'bed','wardrobe','lamp'} <= {v['label'] for v in room['objects']})
            self.assertEqual(len(rooms),30)

    def test_production_multimodal_request_keeps_room_identity_and_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            key=tmp/'key';key.write_text('dummy')
            a=tmp/'a.jpg';b=tmp/'b.jpg';a.write_bytes(b'image a');b.write_bytes(b'image b')
            room={'id':205,'centroid_xy_m':[1,2],'area_m2':15,'objects':[
                {'id':'O042','label':'bed','probability':.77,'center_xyz_m':[1,2,3],
                 'size_xyz_m':[2,1,1],'room_assignment':'center_inside_auto_region'}]}
            expected=json.loads(json.dumps(room))
            clf=QwenRoomSemanticClassifier(20,.65)
            clf.api_key_path=key;clf.cache_dir=tmp/'cache'
            returned={'rooms':[{'room_id':'205','room_type':'bedroom','confidence':.91,
                'evidence_object_ids':['O042'],'evidence_image_indices':[1,2],
                'alternative_types':[],'reason':'bed is visible'}]}
            answer={'model':'qwen3-vl-plus','usage':{'prompt_tokens':100,'completion_tokens':50},
                    'choices':[{'message':{'content':json.dumps(returned)}}]}
            payloads=[]
            def fake_urlopen(request,timeout):
                payloads.append(json.loads(request.data))
                return io.BytesIO(json.dumps(answer).encode())
            with patch('urllib.request.urlopen',side_effect=fake_urlopen):
                decisions,prov=clf.classify([room],{'205':[a,b]})
            self.assertEqual(room,expected)
            self.assertEqual(decisions[0]['room_type'],'bedroom')
            self.assertEqual(decisions[0]['evidence_image_indices'],[1,2])
            self.assertEqual(prov['classifier'],'qwen_vl_multiview')
            content=payloads[0]['messages'][1]['content']
            self.assertEqual(sum(item['type']=='image_url' for item in content),2)
            self.assertIn('O042',content[0]['text'])
            with patch('urllib.request.urlopen',side_effect=AssertionError('cache missed')):
                again, cached=clf.classify([room],{'205':[a,b]})
            self.assertEqual(again,decisions)
            self.assertTrue(cached['calls'][0]['cache_hit'])

if __name__=='__main__':unittest.main()
