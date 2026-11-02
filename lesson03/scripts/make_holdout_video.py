import argparse,json
from pathlib import Path
import cv2
p=argparse.ArgumentParser();p.add_argument('images',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
images=sorted(a.images.glob('*.jpg'))[:60]
out=cv2.VideoWriter(str(a.output),cv2.VideoWriter_fourcc(*'MJPG'),6,(1280,720))
assert out.isOpened()
for path in images:
 image=cv2.imread(str(path));assert image is not None
 out.write(cv2.resize(image,(1280,720)))
out.release();a.output.with_suffix('.json').write_text(json.dumps([p.name for p in images],indent=2))
