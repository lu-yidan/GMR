"""Local interactive G1 browser with serialized, on-demand DuoBox retargeting."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import sys
import threading
from urllib.parse import urlparse, parse_qs

import mujoco
import numpy as np
from duobox_to_g1_pair import ROBOT_XML_DICT, load_pair

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'motion_data/duobox_web'
DATA = ROOT.parent / 'ready_to_react/data/DuoBox/fbx_export_joints'
FILES = sorted(DATA.glob('*.npy'))
MODEL = mujoco.MjModel.from_xml_path(str(ROBOT_XML_DICT['unitree_g1']))
POOL = ThreadPoolExecutor(max_workers=1)
LOCK = threading.Lock()
JOBS = {}
from reactive_live import LIVE


def folder(index, mode):
    if mode == 'reactive_rt3':
        return WEB / 'motions' / f'{index:03d}_reactive_rt3_v1'
    if mode == 'reactive_stream':
        return WEB / 'motions' / f'{index:03d}_reactive_stream_v1'
    if mode == 'body_anchor':
        return WEB / 'motions' / f'{index:03d}_geometry_v10'
    if mode == 'legacy_v5':
        return WEB / 'motions' / f'{index:03d}_geometry_v5'
    if mode == 'geometry':
        return WEB / 'motions' / f'{index:03d}_geometry_v7'
    if index == 4:
        return ROOT / 'motion_data' / ('duobox_004_pair' if mode == 'baseline' else 'duobox_004_continuous')
    return WEB / 'motions' / f'{index:03d}_{mode}'


def ready(index, mode):
    path = folder(index, mode)
    return (path / 'pair_motion.npz').exists() and (path / 'metadata.json').exists() and (mode not in ('reactive_stream','reactive_rt3') or (path / 'pipeline.done').exists())


def prepare(index, mode):
    key = (index, mode)
    try:
        out = folder(index, mode)
        out.mkdir(parents=True, exist_ok=True)
        if mode == 'reactive_rt3':
            deploy=ROOT.parent/'RoboMimic_Deploy-scalebfm'
            job=out/'realtime'
            with (out/'generation.log').open('w') as log:
                subprocess.run(['/home/d080/miniconda3/envs/scalebfm-native/bin/python',str(deploy/'tools/reactive_realtime.py'),'--out',str(job),'--clip',str(index),'--frames','300','--delay-ms','300'],cwd=deploy,stdout=log,stderr=subprocess.STDOUT,check=True)
                subprocess.run([sys.executable,str(ROOT/'scripts/export_realtime_browser.py'),'--job',str(job),'--out',str(out)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
            with LOCK:
                JOBS[key]='ready'
            return
        if mode == 'reactive_stream':
            if (out / 'request.json').exists():
                raise ValueError('已有未完成任务，请检查 generation.log；为保留诊断不自动覆盖。')
            (out / 'request.json').write_text(json.dumps(dict(clip=index,fixed=1,start=0,frames=120,seed=42)))
            with (out / 'generation.log').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'scripts/reactive_g1_pipeline.py'),'--job',str(out)],
                               cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
            with LOCK:
                JOBS[key] = 'ready'
            return
        # A failed/incomplete output is preserved; report it instead of overwriting.
        cmd = [sys.executable, str(ROOT / 'scripts/duobox_to_g1_pair.py'),
               '--input', str(FILES[index]), '--output', str(out), '--no-video']
        if mode == 'body_anchor':
            cmd.append('--body-anchor')
        if mode == 'geometry':
            cmd.append('--geometry')
        if mode == 'continuous':
            cmd.append('--continuous')
        with (out / 'generation.log').open('w') as log:
            subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        with LOCK:
            JOBS[key] = 'ready'
    except Exception as exc:
        with LOCK:
            JOBS[key] = 'error: ' + str(exc)


@lru_cache(maxsize=1)
def scene():
    meshes = []
    for i in range(MODEL.nmesh):
        va, vn = MODEL.mesh_vertadr[i], MODEL.mesh_vertnum[i]
        fa, fn = MODEL.mesh_faceadr[i], MODEL.mesh_facenum[i]
        meshes.append({'vertices': MODEL.mesh_vert[va:va+vn].round(6).reshape(-1).tolist(),
                       'faces': MODEL.mesh_face[fa:fa+fn].reshape(-1).tolist()})
    geoms = []
    for i in range(MODEL.ngeom):
        if MODEL.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH and MODEL.geom_group[i] == 1:
            geoms.append({'body': int(MODEL.geom_bodyid[i]), 'mesh': int(MODEL.geom_dataid[i]),
                          'position': MODEL.geom_pos[i].tolist(), 'quaternion': MODEL.geom_quat[i].tolist()})
    return {'meshes': meshes, 'geoms': geoms, 'bodies': MODEL.nbody}


@lru_cache(maxsize=4)
def animation(index, mode):
    path = folder(index, mode)
    motion = np.load(path / 'pair_motion.npz')
    q = motion['qpos']
    data = mujoco.MjData(MODEL)
    frames = np.empty((len(q), 2, MODEL.nbody, 7))
    for f in range(len(q)):
        for a in range(2):
            data.qpos[:] = q[f, a]
            mujoco.mj_forward(MODEL, data)
            frames[f, a, :, :3] = data.xpos
            frames[f, a, :, 3:] = data.xquat
    meta = json.loads((path / 'metadata.json').read_text())
    return {'frames': frames.round(5).reshape(-1).tolist(), 'count': len(q), 'bodies': MODEL.nbody,
            'metadata': meta, 'center': q[:, :, :3].mean(axis=(0, 1)).tolist()}


@lru_cache(maxsize=1)
def catalog_base():
    rows = []
    for i, file in enumerate(FILES):
        d = np.load(file, allow_pickle=True).item()
        length = min(len(v) for k, v in d.items() if k.endswith('_position'))
        rows.append({'id': i, 'name': file.stem, 'seconds': length / 120})
    return rows


class Handler(BaseHTTPRequestHandler):
    def send(self, body, mime, status=200):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, separators=(',', ':')).encode()
        elif isinstance(body, str):
            body = body.encode()
        compress = len(body) > 4096 and 'gzip' in self.headers.get('Accept-Encoding', '')
        if compress:
            body = gzip.compress(body, compresslevel=3)
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        if compress:
            self.send_header('Content-Encoding', 'gzip')
            self.send_header('Vary', 'Accept-Encoding')
        self.end_headers()
        self.wfile.write(body)

    def selection(self):
        query = parse_qs(urlparse(self.path).query)
        index = int(query.get('id', ['4'])[0])
        mode = query.get('mode', ['body_anchor'])[0]
        if not 0 <= index < len(FILES) or mode not in ('baseline', 'continuous', 'geometry', 'legacy_v5', 'body_anchor', 'reactive_stream', 'reactive_rt3'):
            raise ValueError('Invalid selection')
        return index, mode

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == '/hub':
                return self.send((ROOT/'scripts/boxer_hub.html').read_bytes(),'text/html; charset=utf-8')
            if path == '/live':
                return self.send((ROOT/'scripts/reactive_live.html').read_bytes(),'text/html; charset=utf-8')
            if path == '/live/events':
                return LIVE.events(self)
            if path == '/live/results':
                return self.send(LIVE.results(),'application/json')
            if path == '/':
                return self.send((ROOT / 'scripts/duobox_g1_browser.html').read_bytes(), 'text/html; charset=utf-8')
            if path.startswith('/vendor/') and path[8:] in ('three.module.js', 'three.core.js', 'OrbitControls.js', 'LICENSE'):
                return self.send((WEB / 'vendor' / path[8:]).read_bytes(), 'text/javascript')
            if path == '/catalog':
                return self.send([dict(r, baseline=ready(r['id'], 'baseline'), continuous=ready(r['id'], 'continuous'), geometry=ready(r['id'], 'geometry'))
                                  for r in catalog_base()], 'application/json')
            if path == '/scene':
                return self.send(scene(), 'application/json')
            if path in ('/motion', '/status', '/download'):
                i, mode = self.selection()
                if path == '/status':
                    with LOCK:
                        status = 'ready' if ready(i, mode) else JOBS.get((i, mode), 'missing')
                    info = {'status': status}
                    if mode == 'reactive_stream' and (folder(i,mode)/'metadata.json').exists():
                        info['progress'] = json.loads((folder(i,mode)/'metadata.json').read_text()).get('frames',0)
                    return self.send(info, 'application/json')
                if not ready(i, mode):
                    return self.send({'error': 'Motion not generated yet'}, 'application/json', 404)
                if path == '/download':
                    return self.send((folder(i, mode) / 'pair_motion.npz').read_bytes(), 'application/octet-stream')
                return self.send(animation(i, mode), 'application/json')
            self.send_error(404)
        except (ValueError, KeyError):
            self.send_error(400)
        except Exception as exc:
            self.send({'error': str(exc)}, 'application/json', 500)

    def do_POST(self):
        # Only a same-origin local UI can enqueue work. No file paths or commands are accepted.
        if urlparse(self.path).path in ('/live/start','/live/stop'):
            origin=self.headers.get('Origin')
            if origin and origin != 'http://'+self.headers.get('Host',''):return self.send_error(403)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if length>4096:raise ValueError('Request too large')
                body=json.loads(self.rfile.read(length) or b'{}')
                result=LIVE.start(body,len(FILES)) if self.path=='/live/start' else LIVE.stop()
                return self.send(result,'application/json')
            except (ValueError,KeyError) as exc:
                return self.send({'error':str(exc)},'application/json',400)
        if urlparse(self.path).path != '/generate':
            return self.send_error(404)
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            return self.send_error(403)
        try:
            i, mode = self.selection()
            if mode == 'legacy_v5':
                return self.send({'error': 'v5 仅供已有缓存对比，请选择新修正版生成。'}, 'application/json', 400)
            with LOCK:
                if not ready(i, mode) and JOBS.get((i, mode)) != 'running':
                    JOBS[(i, mode)] = 'running'
                    POOL.submit(prepare, i, mode)
            self.send({'status': 'ready' if ready(i, mode) else 'running'}, 'application/json')
        except ValueError:
            self.send_error(400)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=19093)
    args = parser.parse_args()
    if not (WEB / 'vendor/three.module.js').exists():
        parser.error('Install the local Three.js assets using scripts/setup_duobox_web.py first')
    print(f'Interactive G1 browser: http://127.0.0.1:{args.port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
