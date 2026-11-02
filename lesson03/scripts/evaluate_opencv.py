"""Class-agnostic greedy IoU=0.5 evaluation; this is not the YOLO mAP metric."""
import argparse,json
from pathlib import Path
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('predictions',type=Path);p.add_argument('data',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
def iou(x,y):
 w=max(0,min(x[2],y[2])-max(x[0],y[0]));h=max(0,min(x[3],y[3])-max(x[1],y[1]));inter=w*h
 return inter/max(1e-9,(x[2]-x[0])*(x[3]-x[1])+(y[2]-y[0])*(y[3]-y[1])-inter)
tp=fp=fn=0
for line in a.predictions.read_text().splitlines():
 r=json.loads(line);w,h=Image.open(a.data/'val/images'/r['image']).size;targets=[]
 for row in (a.data/'val/labels'/Path(r['image']).with_suffix('.txt')).read_text().splitlines():
  c,x,y,bw,bh,*_=map(float,row.split());targets.append([(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h])
 used=set()
 for box in r['boxes']:
  candidates=[(iou(box,target),i) for i,target in enumerate(targets) if i not in used]
  score,index=max(candidates,default=(0,-1))
  if score>=.5:used.add(index);tp+=1
  else:fp+=1
 fn+=len(targets)-len(used)
result={'metric':'class-agnostic bbox IoU>=0.5; greedy matching','tp':tp,'fp':fp,'fn':fn,'precision':tp/max(1,tp+fp),'recall':tp/max(1,tp+fn)}
a.output.write_text(json.dumps(result,indent=2));print(result)
