"""Add explicitly reviewed B3 front-plate labels from a fixed simulator camera.
The fixed points are valid ONLY for the supplied unchanged 1440x1080 camera pose.
Simulation validation frames share the same scene; they are not an independent test.
"""
import argparse,json,shutil
from pathlib import Path
import cv2,yaml
p=argparse.ArgumentParser();p.add_argument('subset',type=Path);p.add_argument('video',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
assert not a.output.exists()
for split in ['train','val']:
 for folder in ['images','labels']:
  dest=a.output/split/folder;dest.mkdir(parents=True)
  for f in (a.subset/split/folder).iterdir():
   if f.is_file():(dest/f.name).symlink_to(f.resolve())
video=cv2.VideoCapture(str(a.video));assert video.isOpened();n=0;added={'train':[],'val':[]}
while True:
 ok,im=video.read()
 if not ok:break
 if n%3==0:
  assert im.shape[:2]==(1080,1440)
  split='val' if n%30==0 else 'train';name=f'simulator_{n:04d}';cv2.imwrite(str(a.output/split/'images'/f'{name}.jpg'),im)
  # Reviewed physical front armor: TL,BL,BR,TR. Robot front visibly bears digit 3.
  pts=[688/1440,637/1080,688/1440,661/1080,751/1440,661/1080,751/1440,637/1080]
  row=[2,(688+751)/2/1440,(637+661)/2/1080,63/1440,24/1080,*pts]
  (a.output/split/'labels'/f'{name}.txt').write_text(' '.join(map(str,row))+'\n');added[split].append(n)
 n+=1
cfg=yaml.safe_load((a.subset/'armor.yaml').read_text());cfg['path']=str(a.output.resolve());(a.output/'armor.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));(a.output/'simulation-labels.json').write_text(json.dumps({'source':str(a.video),'reviewed_points_TL_BL_BR_TR':pts,'class_id':2,'class_name':'B3','added_frame_indices':added,'limitation':'fixed camera; only front plate annotated; same-scene validation'},indent=2));print({k:len(v) for k,v in added.items()})
