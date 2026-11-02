"""Linux little-endian binary worker. stdout is reserved for the C++ protocol."""
import os,sys,struct
output=os.fdopen(os.dup(1),'wb',buffering=0)
os.dup2(2,1)
import numpy as np
import torch
from ultralytics import YOLO

def exact(size):
    data=bytearray()
    while len(data)<size:
        block=sys.stdin.buffer.read(size-len(data))
        if not block:raise EOFError
        data.extend(block)
    return data

def main():
    torch.set_num_threads(4)
    model=YOLO(sys.argv[1]);device=os.environ.get('RM_POSE_DEVICE','0' if torch.cuda.is_available() else 'cpu')
    output.write(struct.pack('<I',0x41524d52))
    while True:
        try:height,width,size=struct.unpack('<III',exact(12))
        except EOFError:return
        if not 0<height<=8192 or not 0<width<=8192 or size!=height*width*3:raise ValueError('Invalid frame')
        image=np.frombuffer(exact(size),dtype=np.uint8).reshape(height,width,3)
        result=model.predict(image,imgsz=640,conf=float(os.environ.get('RM_POSE_CONF','.25')),device=device,verbose=False)[0]
        rows=[]
        if result.keypoints is not None:
            for box,kpts in zip(result.boxes,result.keypoints.xy.cpu().numpy()):
                # Source TL,BL,BR,TR -> course TL,TR,BR,BL. Never sort perspective points by x+y.
                pts=kpts[[0,3,2,1]].copy();pts[:,0]=pts[:,0].clip(0,width-1);pts[:,1]=pts[:,1].clip(0,height-1)
                length=(np.linalg.norm(pts[3]-pts[0])+np.linalg.norm(pts[2]-pts[1]))/2
                span=(np.linalg.norm(pts[1]-pts[0])+np.linalg.norm(pts[2]-pts[3]))/2
                if length<1:continue
                rows.append([*pts.flatten(),float(box.conf.item()),float(box.cls.item()),float(span/length>3.2)])
        # Suppress duplicate same-class quadrilaterals from overlapping pose proposals.
        kept=[]
        for row in sorted(rows,key=lambda r:r[8],reverse=True):
            points=np.array(row[:8]).reshape(4,2)
            low,high=points.min(axis=0),points.max(axis=0)
            duplicate=False
            for prior in kept:
                if prior[9]!=row[9]:continue
                q=np.array(prior[:8]).reshape(4,2);ql,qh=q.min(axis=0),q.max(axis=0)
                inter=np.maximum(0,np.minimum(high,qh)-np.maximum(low,ql)).prod()
                union=np.maximum(0,high-low).prod()+np.maximum(0,qh-ql).prod()-inter
                if inter/max(union,1e-9)>.5:duplicate=True;break
            if not duplicate:kept.append(row)
        rows=kept
        output.write(struct.pack('<I',len(rows)))
        for row in rows:output.write(struct.pack('<11f',*row))
if __name__=='__main__':main()
