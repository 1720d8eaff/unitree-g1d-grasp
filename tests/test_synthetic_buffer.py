"""Replay simulated arrivals through the repository's real ActionBuffer."""
import csv
from pathlib import Path
from vla.action_buffer import ActionBuffer, Chunk


def test_synthetic_arrival_replay():
    with (Path(__file__).parent/'fixtures/synthetic_events.csv').open(encoding='utf-8',newline='') as f:
        rows=list(csv.DictReader(f))
    assert {r['load'] for r in rows}=={'low','medium','high'}
    for level in ('low','medium','high'):
        buffer=ActionBuffer(max_age=.15,max_state_error=.1)
        for row in [r for r in rows if r['load']==level]:
            assert row['source']=='synthetic_sdv'
            observed=float(row['observation_timestamp']);start=float(row['inference_start_timestamp']);end=float(row['inference_end_timestamp'])
            assert observed<=start<=end<=float(row['execution_start_timestamp'])
            request=int(row['request_id'])
            accepted,reason=buffer.replace(Chunk(request,observed,(0.,),((1.,),)),(0.,),end)
            if end-observed>.15:
                assert not accepted and reason in ('stale','out_of_order')
            assert isinstance(buffer.needs_replan(),bool)
