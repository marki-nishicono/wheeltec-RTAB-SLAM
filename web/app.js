'use strict';
const $=id=>document.getElementById(id), canvas=$('map'), ctx=canvas.getContext('2d'), fc=$('features'), fx=fc.getContext('2d');
let state={}, map={}, mode=2, scale=65, center=[0,0], yaw=.5, tilt=.75, moving=null, requestBusy=false, lastState=0, gridImage=null, lastGrid=-1, drag=null, cameraZoom=1;
const keys=new Set();
const clientId=crypto.randomUUID();let commandSeq=0;
const controlReasons={another_driver:'其他页面正在驾驶',map_transition:'地图操作中，行驶已锁定',map_session_offline:'地图管理服务离线，行驶已锁定',control_offline:'控制服务离线，正在等待恢复',wheel_offline:'底盘反馈中断，请检查底盘电源 / USB',scan_raw_offline:'激光雷达离线',motion_locked:'车端控制已锁定',imu_calibrating_keep_stationary:'IMU 标定中，请保持车辆静止',base_stale:'底盘反馈超时',scan_stale:'激光雷达数据超时',front_obstacle_or_no_returns:'前方障碍或无有效回波，前进已拦截',rear_obstacle_or_no_returns:'后方障碍或无有效回波，倒车已拦截',deadman_timeout:'等待按键指令',ready:'控制链路正常'};
let controlSession=null,driveError='',controlState={},lastControl=0,controlRtt=0,controlFailures=0;
function controlBlock(){
 if(performance.now()-lastControl>700)return 'control_offline';
 for(const [key,limit] of [['control',.8],['wheel',.6],['scan_raw',.6]])if(!(controlState.ages?.[key]+(performance.now()-lastControl)/1000<limit))return key+'_offline';
 return controlState.drive_block||(!controlState.control?.enabled?'motion_locked':null);
}
function renderControl(){
 renderBattery();
 const block=controlBlock(),c=controlState.control||{};
 $('armed').disabled=Boolean(block);
 if((block||controlSession&&c.session!==controlSession)&&($('armed').checked||moving))stop();
 controlSession=c.session||null;
 const reason=block||c.reason;
 $('controlState').textContent=`${controlReasons[reason]||reason||'等待控制状态'}${driveError?' · '+driveError:''} · ${$('armed').checked?'已使能':'未使能'}\n指令 ${(c.speed||0).toFixed(2)} m/s · 实测轮速 ${(controlState.wheel_speed||0).toFixed(2)} m/s\n前方 ${Number.isFinite(c.front_m)?c.front_m.toFixed(2):'—'} m · 后方 ${Number.isFinite(c.rear_m)?c.rear_m.toFixed(2):'—'} m（仅显示）\n障碍距离自动停车：${c.obstacle_stop_enabled===false?'已关闭':'等待状态'} · 控制延迟 ${controlRtt.toFixed(0)} ms · 最大指令间隔 ${controlState.max_command_gap_ms??0} ms · 看门狗停车 ${c.watchdog_stops??0} 次 · 连接重建 ${controlFailures} 次`;
 if(block){$('status').textContent='行驶不可用 · '+controlReasons[block];$('status').className='tag bad'}
}
function renderResources(){
 const r=state.resources||{};
 const pct=v=>Number.isFinite(v)?v.toFixed(1)+'%':'—';
 $('cpuUsage').textContent=pct(r.cpu_percent);$('gpuUsage').textContent=pct(r.gpu_percent);$('ramUsage').textContent=pct(r.ram_percent);
 for(const [id,key] of [['cpuBar','cpu_percent'],['gpuBar','gpu_percent'],['ramBar','ram_percent']])$(id).value=r[key]||0;
 $('resourceNote').textContent=`RAM ${r.ram_used_mb??'—'} / ${r.ram_total_mb??'—'} MB · GPU 共享内存`;
 $('cameraRate').textContent=`采集 ${r.camera_fps??'—'} fps · H.264 ${r.video?.fps??'—'} fps · ${r.video?.kbps??'—'} kbps · 320×240`;
}
let mapPending=false,lastMapEpoch=null,lastMapList='',mapsLoading=false,mapsRetry=null;
function renderBattery(){
 const b=controlState.battery||{},fresh=b.fresh&&performance.now()-lastControl<2000;
 $('batteryVoltage').textContent=fresh?b.voltage.toFixed(2)+' V':'— V';
 $('batteryNote').textContent=!fresh?'未收到新鲜电压数据':b.warning?'电池电压持续低于 9.9 V 已超过 20 秒，请及时充电！':b.low_seconds>0?`低于 9.9 V 已持续 ${b.low_seconds.toFixed(1)} 秒`:'电池电压正常';
 $('batteryWarning').hidden=!(fresh&&b.warning);
 $('batteryNote').className=fresh&&b.warning?'bad':'hint';
}
function renderWorkflow(){
 const online=lastState>0&&performance.now()-lastState<4000,a=state.ages||{},m=state.map_session||{};
 const sensors=online&&(a.wheel??999)<1&&(a.scan_raw??999)<1&&(a.rgbd??999)<2;
 const checks=[['底盘',(a.wheel??999)<1],['雷达',(a.scan_raw??999)<1],['RGB-D 相机',(a.rgbd??999)<2]];
 $('stepConnect').textContent=online?'1 · 车辆已连接':'1 · 连接车辆：在项目终端运行 bash scripts/start_lab.sh';
 $('stepSensors').textContent='2 · 设备检查：'+(online?checks.map(([n,ok])=>n+(ok?'正常':'无新数据')).join(' / '):'等待车辆连接');
 $('stepMap').textContent='3 · 选择地图：'+(m.busy?m.message:m.phase==='mapping'?'当前地图已就绪，可直接采集；或点击“新建地图并开始”':'新建地图，或选择已保存地图匹配后续建');
 $('stepDrive').textContent='4 · 采集环境：'+(sensors&&!m.locked?'勾选行驶使能，W/S 前后、A/D 转向，空格停车':'等待设备就绪与地图初始化完成');
 $('stepSave').textContent='5 · 结束本次采集：空格停车 → 填写名称 → 保存当前地图；下次从已保存地图继续';
 $('deviceStatus').textContent=!online?'车辆未连接。页面服务已打开，但需要建立 SSH 连接并启动车端服务。':!sensors?'有设备尚未提供数据；打开画面不会启动已断开的相机。':'采集与建图在车端运行；相机画面按需开启，不影响建图。';
 for(const id of ['newMap','saveMap','continueMap','deleteMap'])if(!online)$(id).disabled=true;
}
function renderMapSession(){
 const m=state.map_session||{};
 const busy=mapPending||m.busy;
 $('saveMap').disabled=busy||m.phase!=='mapping';$('newMap').disabled=busy;
 $('continueMap').disabled=busy||!$('savedMaps').value||$('savedMaps').selectedOptions[0]?.dataset.empty==='true';
 $('deleteMap').disabled=busy||!$('savedMaps').value;
 const match=m.match?`\n已匹配节点 ${m.match.node_id} · (${m.match.x.toFixed(2)}, ${m.match.y.toFixed(2)}) m · 航向 ${(m.match.yaw*180/Math.PI).toFixed(1)}°`:'';
 $('mapSessionState').textContent=`当前：${m.active_label||'—'}\n${m.message||'等待状态'}${match}${m.last_saved?'\n最近保存：'+m.last_saved.label:''}`;
 $('mapSessionState').className=m.phase==='error'?'bad':m.locked?'hint':'ok';
 if(lastMapEpoch!==m.epoch){
  if(lastMapEpoch!==null){keys.clear();moving=null;$('armed').checked=false;map={};gridImage=null;lastGrid=-1;}
  lastMapEpoch=m.epoch;
 }
 const changed=m.last_saved?.id||'';
 if(changed!==lastMapList){lastMapList=changed;refreshMaps()}
}
async function refreshMaps(){
 if(mapsLoading)return;mapsLoading=true;clearTimeout(mapsRetry);
 try{
  const d=await api('maps'),s=$('savedMaps'),selected=s.value;s.replaceChildren();
  for(const m of d.maps){const o=document.createElement('option');o.value=m.id;o.dataset.empty=String(m.node_count===0);o.textContent=`${m.label} · ${m.node_count} 节点 · ${new Date(m.created*1000).toLocaleString()}`;s.append(o)}
  if(!d.maps.length){const o=document.createElement('option');o.value='';o.textContent='尚无已保存地图';s.append(o)}
  if(d.maps.some(m=>m.id===selected))s.value=selected;
  $('mapActionError').textContent='';renderMapSession();
 }catch(e){
  const s=$('savedMaps'),o=document.createElement('option');o.value='';o.textContent='读取失败 · 等待车辆连接，可点击刷新列表';s.replaceChildren(o);
  $('continueMap').disabled=true;$('deleteMap').disabled=true;
  $('mapActionError').textContent='读取地图失败：'+e.message;mapsRetry=setTimeout(refreshMaps,5000);
 }finally{mapsLoading=false}
}
async function mapAction(action){
 if(mapPending||state.map_session?.busy)return;
 if(action==='delete'&&!confirm('永久删除所选已保存地图：'+$('savedMaps').selectedOptions[0]?.textContent+'？当前工作地图副本保留。'))return;
 stop();mapPending=true;renderMapSession();$('mapActionError').textContent='';
 try{
  const r=await fetch('/api/map-session',{method:'POST',headers:{'Content-Type':'application/json','X-Lab-Control':'deadman-v1'},body:JSON.stringify({action,label:$('mapName').value,map_id:$('savedMaps').value,request_id:crypto.randomUUID()}),signal:AbortSignal.timeout(5000)});
  const d=await r.json();if(!r.ok)throw Error(d.error||r.status);
  if(action==='delete'){await refreshMaps();$('mapActionError').textContent='已删除：'+d.deleted_label;return}
  state.map_session={...(state.map_session||{}),busy:true,locked:true,message:'地图操作已提交'};
 }catch(e){$('mapActionError').textContent='操作提交异常，请先查看状态再重试：'+e.message}
 finally{mapPending=false;renderMapSession()}
}
$('saveMap').onclick=()=>mapAction('save');$('newMap').onclick=()=>mapAction('new');$('continueMap').onclick=()=>mapAction('continue');$('refreshMaps').onclick=refreshMaps;$('savedMaps').onchange=renderMapSession;
 $('deleteMap').onclick=()=>mapAction('delete');
async function api(path){const r=await fetch('/api/'+path,{cache:'no-store',signal:AbortSignal.timeout(4000)});if(!r.ok)throw Error(r.status===502?'车辆未连接或车端服务未启动':r.status===503?'设备尚未就绪':'服务返回 '+r.status);return r.json()}
function resize(c){let d=window.devicePixelRatio||1;c.width=c.clientWidth*d;c.height=c.clientHeight*d;c.getContext('2d').setTransform(d,0,0,d,0,0)}
function project(p){
 const w=canvas.clientWidth,h=canvas.clientHeight;
 if(mode===4){
  const c=state.camera_view,t=c?.map_from_camera;
  if(!t||(state.ages?.camera_pose??999)>2)return null;
  const d=[p[0]-t[0][3],p[1]-t[1][3],(p[2]||0)-t[2][3]];
  const q=[0,1,2].map(j=>d.reduce((v,x,i)=>v+x*t[i][j],0));
  if(q[2]<.08)return null;
  const f=Math.min(w/c.width,h/c.height)*cameraZoom;
  return [w/2+(c.k[0]*q[0]/q[2]+c.k[2]-c.width/2)*f,h/2+(c.k[4]*q[1]/q[2]+c.k[5]-c.height/2)*f,q[2]];
 }
 let x=p[0]-center[0],y=p[1]-center[1],z=p[2]||0;
 if(mode===3){const a=x*Math.cos(yaw)-y*Math.sin(yaw),b=x*Math.sin(yaw)+y*Math.cos(yaw);x=a;y=b*Math.cos(tilt)+z*Math.sin(tilt)}
 return [w/2+x*scale,h/2-y*scale,0];
}
function path(points,color){ctx.strokeStyle=color;ctx.lineWidth=1.6;ctx.beginPath();let connected=false;for(const p of points){const q=project(p);if(!q){connected=false;continue}if(connected)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);connected=true}ctx.stroke()}
function draw(){
 resize(canvas);const w=canvas.clientWidth,h=canvas.clientHeight;
 ctx.fillStyle='#071019';ctx.fillRect(0,0,w,h);
 if(mode!==4){for(let x=-30;x<=30;x++){path([[x,-30,0],[x,30,0]],'#122b37');path([[-30,x,0],[30,x,0]],'#122b37')}}
 if(mode===2&&gridImage&&map.grid){const g=map.grid,o=g.origin,p=project([o[0][3],o[1][3],0]);ctx.save();ctx.translate(p[0],p[1]);ctx.rotate(-Math.atan2(o[1][0],o[0][0]));ctx.scale(scale*g.resolution,-scale*g.resolution);ctx.imageSmoothingEnabled=false;ctx.globalAlpha=.6;ctx.drawImage(gridImage,0,0);ctx.restore()}
 let points=(map.cloud||[]).map(p=>({p,q:project(p)})).filter(o=>o.q&&o.q[0]>=0&&o.q[0]<w&&o.q[1]>=0&&o.q[1]<h);
 if(mode===4)points.sort((a,b)=>b.q[2]-a.q[2]);
 for(const {p,q} of points){ctx.fillStyle=`rgb(${p[3]},${p[4]},${p[5]})`;const size=mode===4?Math.max(2,Math.min(6,4/q[2])):mode===3?2:1.5;ctx.fillRect(q[0],q[1],size,size)}
 if($('showPaths').checked){const g=map.graph||{};for(const e of map.links||[]){if(!g[e.a]||!g[e.b])continue;const a=g[e.a],b=g[e.b];path([[a[0][3],a[1][3],a[2][3]],[b[0][3],b[1][3],b[2][3]]],e.type===0?'#54d4b4':'#ff4668')}
  if(mode!==4)for(const [key,color] of [['wheel','#ad92ef'],['ekf','#faaf62']])path((state.trajectories?.[key]||[]).map(p=>[p[1],p[2],0]),color);
 }
 if($('showScan').checked){ctx.fillStyle='#65efff';for(const p of state.scan||[]){const q=project(p);if(q)ctx.fillRect(q[0],q[1],2,2)}}
 if(state.pose&&mode!==4){const p=project(state.pose);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fill();if(mode===3)path([state.pose.slice(0,3),[state.pose[0],state.pose[1],state.pose[2]+.5]],'#ffe08a')}
 ctx.fillStyle='#91aabd';ctx.font='12px system-ui';ctx.fillText(mode===4?'车载相机视角 · 随车辆移动 · 光学投影':mode===3?'3D · Z 轴朝上 · RGB-D 关键帧点云':'2D · M10 栅格 + RGB-D 点云',15,23);
 if(mode===4&&(!state.camera_view||(state.ages?.camera_pose??999)>2))ctx.fillText('等待相机位姿更新',15,45);
}
function decodeGrid(){let g=map.grid;if(!g){gridImage=null;lastGrid=-1;return}if(lastGrid===map.revision)return;lastGrid=map.revision;let raw=atob(g.data),c=document.createElement('canvas');c.width=g.w;c.height=g.h;let x=c.getContext('2d'),im=x.createImageData(g.w,g.h);for(let i=0;i<raw.length;i++){let v=raw.charCodeAt(i);if(v===255)continue;let a=i*4,gray=v>50?230:43;im.data.set([gray,gray,gray,255],a)}x.putImageData(im,0,0);gridImage=c}
function fit(){if(mode===4){cameraZoom=1;draw();return}let ps=Object.values(map.graph||{}).map(t=>[t[0][3],t[1][3]]);if(map.cloud?.length)ps=map.cloud.filter((_,i)=>i%20===0);if(!ps.length)return;let xs=ps.map(p=>p[0]),ys=ps.map(p=>p[1]);let a=Math.min(...xs),b=Math.max(...xs),c=Math.min(...ys),d=Math.max(...ys);center=[(a+b)/2,(c+d)/2];scale=Math.min(canvas.clientWidth/Math.max(4,b-a+1),canvas.clientHeight/Math.max(4,d-c+1));draw()}
async function pollState(){try{state=await api('state');lastState=performance.now();let age=state.ages?.rtabmap??999;$('status').textContent=state.map_session?.locked?'地图操作 · '+state.map_session.message:age<4?'在线 · 正在建图':'在线 · SLAM 数据待更新';$('status').className='tag '+(age<4?'ok':'bad');$('nodes').textContent=state.graph_nodes??0;$('points').textContent=state.cloud_points??0;$('loop').textContent=state.loop_id||'无';let stats=state.stats||{};let t=Object.entries(stats).find(([k])=>/Timing\/Total\/ms/.test(k));$('timing').textContent=t?Number(t[1]).toFixed(1):'—';renderControl();renderResources();renderMapSession();renderWorkflow();$('log').textContent=`工作记忆 ${state.wm_nodes??0} 节点\n缓存关键帧 ${state.cached_keyframes??0}\n深度有效率 ${((state.depth_valid_fraction||0)*100).toFixed(1)}%\nRGB-D 时间差 ${state.rgbd_skew_ms??'—'} ms\n地图准备 ${state.map_prepare_ms??'—'} ms\n数据年龄 ${JSON.stringify(state.ages)}\n\n${Object.entries(stats).filter(([k])=>/Timing|Inliers|Features|Words|Memory/.test(k)).slice(0,16).map(([k,v])=>k+': '+Number(v).toFixed(2)).join('\n')}`;draw()}catch(e){$('status').textContent='地图数据传输中断（控制独立检测）';$('status').className='tag bad'}renderWorkflow();setTimeout(pollState,1000)}
async function pollMap(){if(controlRtt>250||moving){setTimeout(pollMap,3000);return}try{map=await api('map');decodeGrid();draw()}catch(e){}setTimeout(pollMap,3000)}
function imageFrom(j){return new Promise(resolve=>{let i=new Image();i.onload=()=>resolve(i);i.src='data:image/jpeg;base64,'+j})}
let featuresEnabled=false;$('openFeatures').onclick=()=>{featuresEnabled=!featuresEnabled;$('openFeatures').textContent=featuresEnabled?'关闭特征匹配画面':'打开特征匹配画面';if(!featuresEnabled){fx.clearRect(0,0,fc.width,fc.height);$('featureLabel').textContent='特征画面已关闭'}};
async function pollFeatures(){if(!featuresEnabled||document.hidden||controlRtt>250||moving){setTimeout(pollFeatures,5000);return}try{let f=await api('features');if(!f.current){resize(fc);fx.clearRect(0,0,fc.width,fc.height);$('featureLabel').textContent='等待当前地图关键帧'}if(f.current){let a=await imageFrom(f.current.jpeg),b=f.previous?await imageFrom(f.previous.jpeg):null;resize(fc);fx.fillStyle='#071019';fx.fillRect(0,0,fc.clientWidth,fc.clientHeight);let s=Math.min(fc.clientWidth/(a.width+(b?.width||0)),fc.clientHeight/a.height);fx.drawImage(a,0,0,a.width*s,a.height*s);if(b)fx.drawImage(b,a.width*s,0,b.width*s,b.height*s);fx.strokeStyle='#52efbb';for(let p of f.current.keypoints){fx.beginPath();fx.arc(p[0]*s,p[1]*s,2,0,7);fx.stroke()}if(b){fx.strokeStyle='#ffd36b88';for(let p of f.pairs){fx.beginPath();fx.moveTo(p[0][0]*s,p[0][1]*s);fx.lineTo((p[1][0]+a.width)*s,p[1][1]*s);fx.stroke()}}$('featureLabel').textContent=`关键帧 ${f.current.id} ↔ ${f.previous?.id??'无'} · ${f.current.keypoints.length} 个实际特征 · ${f.pairs.length} 条共有词候选 · 接受闭环 ${f.accepted_loop_id||'无'}（黄色连线不是几何验证内点）`}}catch(e){}setTimeout(pollFeatures,5000)}
let driveChannel=null;
function drive(v,s,emergency=false){if(!driveChannel?.send(v,s,controlState.drive_token,emergency,controlState.emergency_epoch)){if(v){moving=null;keys.clear();$('armed').checked=false;driveError='控制长连接不可用'}}}
function stop(force=false){const active=Boolean(moving)||$('armed').checked;moving=null;keys.clear();$('armed').checked=false;if(active||force)drive(0,0,force)}
function startControl(){
 driveChannel=new DriveChannel(d=>{
  if(controlState.emergency_epoch!==undefined&&controlState.emergency_epoch!==d.emergency_epoch){moving=null;keys.clear();$('armed').checked=false}
  controlState=d;lastControl=performance.now();
  if(Number.isFinite(d.echo))controlRtt=performance.now()-d.echo;
  driveError=d.socket_error||'';
  if(d.driver_other)controlState.drive_block='another_driver';
  renderControl();
 },()=>{moving=null;keys.clear();$('armed').checked=false;lastControl=0;controlFailures++});
 setInterval(()=>driveChannel.ping(),200);
}
function keyboardDrive(){
 const forward=Number(keys.has('KeyW'))-Number(keys.has('KeyS'));
 const steering=Number(keys.has('KeyA'))-Number(keys.has('KeyD'));
 moving=forward?[forward*.10,steering*.30]:null;
 drive(...(moving||[0,0]));
}
window.addEventListener('keydown',e=>{
 if(e.code==='Space'){e.preventDefault();stop(true);return}
 if(!['KeyW','KeyA','KeyS','KeyD'].includes(e.code)||/INPUT|TEXTAREA|SELECT/.test(e.target.tagName)&&e.target.type!=='checkbox')return;
 e.preventDefault();
 if(!$('armed').checked||controlBlock()||document.hidden)return;
 keys.add(e.code);keyboardDrive();
});
window.addEventListener('keyup',e=>{
 if(keys.delete(e.code)){e.preventDefault();keyboardDrive()}
});
document.querySelectorAll('[data-v]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();if(!$('armed').checked||controlBlock())return;b.setPointerCapture(e.pointerId);moving=[+b.dataset.v,+b.dataset.s];drive(...moving)};b.onpointerup=b.onpointercancel=b.onlostpointercapture=()=>{moving=null;drive(0,0)}});
setInterval(()=>{renderControl();if(moving){if(document.hidden||controlBlock()||!$('armed').checked)stop();else drive(...moving)}},50);
$('stop').onclick=()=>stop(true);window.addEventListener('blur',()=>stop());document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});window.addEventListener('pagehide',()=>stop());$('armed').onchange=()=>{if(!$('armed').checked)stop()};
for(let n of [2,3,4])$('view'+n).onclick=()=>{mode=n;for(let v of [2,3,4])$('view'+v).classList.toggle('active',n===v);draw()};$('fit').onclick=fit;canvas.onwheel=e=>{e.preventDefault();if(mode===4)cameraZoom=Math.max(.5,Math.min(3,cameraZoom*Math.exp(-e.deltaY*.001)));else scale=Math.max(5,Math.min(500,scale*Math.exp(-e.deltaY*.001)));draw()};canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointermove=e=>{if(!drag||mode===4)return;let dx=e.clientX-drag[0],dy=e.clientY-drag[1];if(mode===3){yaw+=dx*.01;tilt=Math.max(0,Math.min(1.5,tilt+dy*.01))}else{center[0]-=dx/scale;center[1]+=dy/scale}drag=[e.clientX,e.clientY];draw()};canvas.onpointerup=()=>{drag=null};window.onresize=draw;startControl();pollState();pollMap();pollFeatures();refreshMaps();

setInterval(renderWorkflow,1000);
