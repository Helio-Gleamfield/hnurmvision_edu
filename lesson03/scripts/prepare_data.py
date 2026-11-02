"""Validate original pose labels, then make a deterministic, stratified small split."""
import argparse, collections, hashlib, json, random, shutil
from pathlib import Path
import yaml
NAMES = ['B1','B2','B3','B4','B5','BO','BS','R1','R2','R3','R4','R5','RO','RS']
def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--train',type=int,default=1200);p.add_argument('--val',type=int,default=240);a=p.parse_args()
    if a.output.exists(): raise SystemExit('Output already exists; choose a new directory')
    rng=random.Random(20261102); report={}; seen=set();manifest={}
    for split,limit in [('train',a.train),('val',a.val)]:
        groups=collections.defaultdict(list); counts=collections.Counter()
        for im in sorted((a.source/split/'images').glob('*')):
            if im.suffix.lower() not in ['.jpg','.png','.jpeg']:continue
            label=a.source/split/'labels'/f'{im.stem}.txt'
            rows=[list(map(float,r.split())) for r in label.read_text().splitlines() if r.strip()]
            for r in rows:
                assert len(r)==13 and r[0]==int(r[0]) and 0<=r[0]<14 and all(0<=v<=1 for v in r[1:]),label
                counts[int(r[0])]+=1
            groups[int(rows[0][0]) if rows else -1].append((im,label))
        for group in groups.values():rng.shuffle(group)
        selected=[]
        while len(selected)<limit and any(groups.values()):
            for k in sorted(groups):
                if groups[k] and len(selected)<limit:
                    im,lab=groups[k].pop();digest=hashlib.sha256(im.read_bytes()).hexdigest()
                    if digest not in seen:seen.add(digest);selected.append((im,lab,digest))
        for folder in ['images','labels']:(a.output/split/folder).mkdir(parents=True)
        manifest[split]=[]
        for im,lab,digest in selected:
            shutil.copy2(im,a.output/split/'images'/im.name);shutil.copy2(lab,a.output/split/'labels'/lab.name)
            manifest[split].append({'image':im.name,'sha256':digest})
        report[split]={'selected':len(selected),'original_instances':dict(counts)}
    config={'path':str(a.output.resolve()),'train':'train/images','val':'val/images','kpt_shape':[4,2],'flip_idx':[3,2,1,0],'names':dict(enumerate(NAMES))}
    (a.output/'armor.yaml').write_text(yaml.safe_dump(config,sort_keys=False));(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2));(a.output/'audit.json').write_text(json.dumps(report,indent=2));print(report)
if __name__=='__main__':main()
