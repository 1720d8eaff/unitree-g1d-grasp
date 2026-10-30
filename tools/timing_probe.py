"""Measure OS scheduling at the DDS backend period, without loading SDK or motors."""
import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=500);p.add_argument('--output',default='artifacts/timing');a=p.parse_args()
    if a.steps<2:raise ValueError('need at least two ticks')
    backend=ROOT/'onboard/trajectory_bridge/g1_d_trajectory_bridge/trajectory_backend.py'
    dt=next(ast.literal_eval(n.value) for n in ast.parse(backend.read_text(encoding='utf-8')).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='CONTROL_DT' for t in n.targets))
    deadline=time.perf_counter();rows=[]
    for i in range(a.steps):
        remaining=deadline-time.perf_counter()
        if remaining>0:time.sleep(remaining)
        now=time.perf_counter();rows.append([i,deadline,now,now-deadline]);deadline+=dt
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    with (out/'ticks.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['tick','scheduled_s','actual_s','lateness_s']);w.writerows(rows)
    late=sorted(x[3] for x in rows)
    report={'source':'host_scheduling_probe','backend_sha256':hashlib.sha256(backend.read_bytes()).hexdigest(),'target_period_s':dt,'ticks':len(rows),'lateness_p50_s':late[int(.5*(len(late)-1))],'lateness_p95_s':late[int(.95*(len(late)-1))],'over_period_count':sum(x>dt for x in late),'elapsed_s':rows[-1][2]-rows[0][2],'scope':'OS sleep scheduling only; no DDS publish or robot movement'}
    (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))

if __name__=='__main__':main()
