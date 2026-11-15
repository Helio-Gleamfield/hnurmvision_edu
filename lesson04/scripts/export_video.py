"""Encode the actual annotated camera frames using their acquisition timestamps."""
import argparse,csv,os,shutil,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('recording',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.recording=a.recording.resolve();a.output=a.output.resolve()
ff=os.environ.get('FFMPEG') or shutil.which('ffmpeg')
if not ff:
 choices=list(Path('/home/helio/RM_Lesson03_Work/media-tools/imageio_ffmpeg/binaries').glob('ffmpeg-linux-*'))
 if choices:ff=str(choices[0])
if not ff:raise SystemExit('Set FFMPEG to an available ffmpeg executable')
rows=list(csv.DictReader((a.recording/'frames.csv').open()));assert len(rows)>1
concat=a.recording/'frames.txt'
with concat.open('w') as f:
 for i,row in enumerate(rows):
  name=row['filename'];assert name.replace('.jpg','').isdigit()
  f.write("file 'frames/"+name+"'\n")
  duration=(int(rows[i+1]['acquisition_ros_ns'])-int(row['acquisition_ros_ns']))/1e9 if i+1<len(rows) else .05
  f.write(f'duration {max(.001,duration):.9f}\n')
 f.write("file 'frames/"+rows[-1]['filename']+"'\n")
a.output.parent.mkdir(parents=True,exist_ok=True)
subprocess.run([ff,'-hide_banner','-nostdin','-n','-f','concat','-safe','0','-i',str(concat),'-an','-vf','fps=30','-c:v','libx264','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],check=True)
print(a.output)
