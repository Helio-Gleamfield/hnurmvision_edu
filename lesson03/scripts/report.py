"""Evaluate trained weights, save confidence-labelled holdout predictions and loss/mAP curves."""
import argparse,json,shutil,csv
from pathlib import Path
import torch
from ultralytics import YOLO
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);p.add_argument('--data',required=True,type=Path);p.add_argument('--out',required=True,type=Path);a=p.parse_args();a.run=a.run.resolve();a.data=a.data.resolve();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=True);torch.set_num_threads(4)
 model=YOLO(a.run/'weights/best.pt');m=model.val(data=str(a.data),device=0,imgsz=640,plots=True,project=str(a.out),name='validation',exist_ok=True)
 (a.out/'metrics.json').write_text(json.dumps({'box_map50':m.box.map50,'box_map50_95':m.box.map,'pose_map50':m.pose.map50,'pose_map50_95':m.pose.map,'precision':m.box.mp,'recall':m.box.mr,'class_indices':m.box.ap_class_index.tolist(),'class_box_map50_95':m.box.maps.tolist()},indent=2))
 for name in ['best.pt','last.pt']:shutil.copy2(a.run/'weights'/name,a.out/name)
 for name in ['results.csv','results.png','args.yaml','environment.json']:
  if (a.run/name).exists():shutil.copy2(a.run/name,a.out/name)
 images=sorted((a.data.parent/'val/images').glob('*.jpg'))
 model.predict(source=[str(p) for p in images[:24]],conf=.25,device=0,save=True,show_conf=True,project=str(a.out),name='predictions',exist_ok=True)
 with (a.run/'results.csv').open() as f:rows=[{k.strip():float(v) for k,v in r.items()} for r in csv.DictReader(f)]
 fig,axes=plt.subplots(1,2,figsize=(13,4))
 for k in rows[0]:
  if k.startswith('train/') and 'loss' in k:axes[0].plot([r['epoch'] for r in rows],[r[k] for r in rows],label=k)
  if 'mAP' in k:axes[1].plot([r['epoch'] for r in rows],[r[k] for r in rows],label=k)
 for ax,title in zip(axes,['Training losses','Validation mAP']):ax.set_title(title);ax.set_xlabel('Epoch');ax.legend(fontsize=8);ax.grid(alpha=.3)
 fig.tight_layout();fig.savefig(a.out/'training-curves.png',dpi=160)
if __name__=='__main__':main()
