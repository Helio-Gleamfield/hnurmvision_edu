"""Reproducible small-set YOLO26 pose training. No implicit full-dataset training."""
import argparse,json,platform,time
from pathlib import Path
from ultralytics import YOLO
import torch,ultralytics

def main():
 p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--project',default='lesson03/runs');p.add_argument('--name',default='armor26');p.add_argument('--epochs',type=int,default=40);p.add_argument('--device',default='0');p.add_argument('--model',default='yolo26n-pose.pt');a=p.parse_args()
 torch.set_num_threads(4);start=time.time()
 model=YOLO(a.model)
 model.train(data=a.data,epochs=a.epochs,imgsz=640,batch=16,device=a.device,workers=2,seed=20261102,deterministic=True,project=str(Path(a.project).resolve()),name=a.name,exist_ok=False,patience=15,plots=True,save=True,amp=False,close_mosaic=5,fliplr=0.5,flipud=0,degrees=10,translate=.1,scale=.3)
 out=Path(model.trainer.save_dir)
 (out/'environment.json').write_text(json.dumps({'torch':torch.__version__,'ultralytics':ultralytics.__version__,'device':torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU','seconds':time.time()-start,'python':platform.python_version()},indent=2))
if __name__=='__main__':main()
