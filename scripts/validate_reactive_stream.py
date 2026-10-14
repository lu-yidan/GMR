"""Validate causal boundaries, coordinate conversion and physical rollout artifacts."""
import argparse
import json
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser(); p.add_argument('job',type=Path); a=p.parse_args()
job=a.job; request=json.loads((job/'request.json').read_text())
root=Path(__file__).resolve().parents[1]
files=sorted((root.parent/'ready_to_react/data/DuoBox/fbx_export_joints').glob('*.npy'))
raw=np.load(files[request['clip']],allow_pickle=True).item()
names=sorted(k[:-9] for k in raw if k.endswith('_position'))
previous=None
for path in sorted(job.glob('human_*.npz')):
    end=int(path.stem.split('_')[1]); data=np.load(path)
    assert int(data['observed_until']) == (3 if end==4 else end-5)
    assert data['positions'].shape == (end,2,21,3)
    if previous is not None:
        np.testing.assert_array_equal(data['positions'][:len(previous)],previous)
    previous=data['positions']
    frames=(np.arange(end)+request['start'])*4+4
    np.testing.assert_allclose(data['positions'][:,1],raw[names[request['fixed']]+'_position'][frames],atol=1e-5)
    rots=data['rotations']
    np.testing.assert_allclose(np.linalg.det(rots),1,atol=1e-4)
actual=np.load(job/'physical_50hz.npz')['qpos']
assert np.isfinite(actual).all()
np.testing.assert_allclose(np.linalg.norm(actual[:,3:7],axis=1),1,atol=1e-5)
meta=json.loads((job/'metadata.json').read_text())
assert meta['failure'] is None, meta['failure']
assert not meta['online_realtime'] and not meta['upper_actual_state_feedback']
assert (job/'pipeline.done').exists()
print('PASS:',len(previous),'human frames;',len(actual),'physical frames; fixed opponent exact, immutable prefix, history-only prediction boundaries')
