"""Local live telemetry and experiment controls; never connects to robot DDS."""
import json,os,signal,socket,subprocess,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
DEPLOY=ROOT.parent/'RoboMimic_Deploy-scalebfm'
class Live:
    def __init__(self):
        self.cv=threading.Condition();self.version=0;self.latest={'status':'idle'};self.process=None;self.job=None;self.runs=[]
        for path in sorted((DEPLOY/'outputs/live_runs').glob('*/report.json'))[-20:]:
            try:self.runs.append(dict(run=path.parent.name,job=str(path.parent),status='finished',report=json.loads(path.read_text())))
            except (ValueError,OSError):pass
        self.udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.udp.bind(('127.0.0.1',0));self.port=self.udp.getsockname()[1]
        threading.Thread(target=self.receive,daemon=True).start()
    def publish(self,value):
        with self.cv:self.latest=value;self.version+=1;self.cv.notify_all()
    def receive(self):
        while True:
            raw,_=self.udp.recvfrom(65535)
            try:value=json.loads(raw)
            except (ValueError,UnicodeDecodeError):continue
            if self.job and value.get('run')==self.job.name:
                self.publish(dict(value,status='running'))
    def start(self,body,count):
        clip=int(body.get('clip',4));seconds=int(body.get('seconds',20));mode=body.get('mode','normal')
        if not 0<=clip<count or not 4<=seconds<=60 or mode not in ('normal','freeze','delay'):raise ValueError('Invalid clip, duration or mode')
        with self.cv:
            if self.process and self.process.poll() is None:raise ValueError('请先停止当前实验')
            self.job=DEPLOY/'outputs/live_runs'/f'{time.time_ns()}_{clip:03d}_{mode}'
            self.job.parent.mkdir(parents=True,exist_ok=True)
            log=self.job.with_suffix('.log').open('w')
            cmd=['/home/d080/miniconda3/envs/scalebfm-native/bin/python',str(DEPLOY/'tools/reactive_realtime.py'),'--out',str(self.job),'--clip',str(clip),'--frames',str(seconds*30),'--opponent-mode',mode,'--telemetry-port',str(self.port)]
            self.process=subprocess.Popen(cmd,cwd=DEPLOY,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
            self.publish(dict(status='starting',run=self.job.name,mode=mode,clip=clip,message='加载模型与预热，完成后自动实时显示'))
            threading.Thread(target=self.watch,args=(self.process,self.job),daemon=True).start()
        return {'status':'starting','run':self.job.name}
    def watch(self,process,job):
        code=process.wait();report=job/'report.json'
        result=json.loads(report.read_text()) if report.exists() else None
        row=dict(run=job.name,job=str(job),status='finished' if result else ('stopped' if code<0 else 'error'),report=result)
        self.runs.append(row)
        if self.job==job:self.publish(dict(self.latest,status=row['status'],result={k:v for k,v in (result or {}).items() if k not in ('ticks','chunks')}))
    def stop(self):
        with self.cv:
            if self.process and self.process.poll() is None:os.killpg(self.process.pid,signal.SIGTERM)
        return {'status':'stopping'}
    def results(self):
        import numpy as np
        rows=[]
        for row in self.runs[-20:]:
            r=row['report'];item=dict(run=row['run'],status=row['status'])
            if r:
                req=json.loads((Path(row['job'])/'request.json').read_text())
                item.update(clip=req['clip'],mode=r['opponent_mode'],completed=r['completed'],holds=r['current_target_hold_frames'],misses=r['deadline_misses'],rtf=r['actual_realtime_factor'])
                if item['mode']!='normal':
                    baselines=[x for x in self.runs if x['report'] and x['report'].get('opponent_mode')=='normal' and json.loads((Path(x['job'])/'request.json').read_text())['clip']==req['clip']]
                    if baselines:
                        b=np.load(Path(baselines[-1]['job'])/'human.npz')['positions'][:,0];v=np.load(Path(row['job'])/'human.npz')['positions'][:,0];n=min(len(b),len(v));cut=int(r['intervention_s']*30)
                        item['common_prefix_max_m']=float(np.abs(b[:min(cut,n)]-v[:min(cut,n)]).max())
                        if n>cut+4:item['response_delta_cm']=float(np.linalg.norm(b[cut+4:n][:,[8,12]]-v[cut+4:n][:,[8,12]],axis=-1).mean()*100)
            rows.append(item)
        return rows
    def events(self,handler):
        handler.send_response(200);handler.send_header('Content-Type','text/event-stream');handler.send_header('Cache-Control','no-cache');handler.end_headers()
        version=-1
        try:
            while True:
                with self.cv:
                    if version==self.version:self.cv.wait(timeout=1)
                    version=self.version;value=dict(self.latest)
                handler.wfile.write(('data: '+json.dumps(value,separators=(',',':'))+'\n\n').encode());handler.wfile.flush()
        except (BrokenPipeError,ConnectionResetError):pass
LIVE=Live()
