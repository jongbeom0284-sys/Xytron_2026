const host=location.hostname;
const ros=new ROSLIB.Ros({url:`ws://${host}:9090`});
const rosState=document.querySelector('#ros-state');
const grid=document.querySelector('#cctv-grid');
const dialog=document.querySelector('#device-dialog');
const dialogContent=document.querySelector('#device-dialog-content');
const recoveryDialog=document.querySelector('#recovery-dialog');
const recoveryDialogContent=document.querySelector('#recovery-dialog-content');
const deviceLabels={camera:'카메라',lidar:'LiDAR',ultrasonic:'초음파',imu:'IMU'};
const deviceOrder=['camera','lidar','ultrasonic','imu'];
const recoveryLabels={camera:'카메라',lane:'차선 인식(YOLOPv2)',video:'영상 서버',lidar:'LiDAR',ultrasonic:'초음파',imu:'IMU'};
const recoveryOrder=['camera','lane','video','lidar','ultrasonic','imu'];
const viewLabels={camera:'전방 카메라',lane:'차선 + 신호등 인식',lidar:'LiDAR 점 화면',ultrasonic:'초음파 배치',imu:'IMU 자세',system:'차량 상태',drive:'주행 테스트',empty:'비어 있음'};
const defaultSlots=['camera','lane','lidar','ultrasonic','imu','system','empty','empty','empty'];
const ultraIndex={left:0,right:4,rearLeft:7,rear:6,rearRight:5};
let deviceApiState={};
let hardwareState={};
let latestScan=null;
let latestUltrasonic={};
let latestImu='-';
let latestMotor='확인 중';
let latestHost='kobot';
let latestPower='대기 중';
let latestLane={state:'WAITING',mode:'WAITING',confidence:0,center_offset_px:0};
let latestTraffic={state:'none',stable:'none',ko:'없음',boxes:[],lamps:0,ms:0,w:640,h:480};
let latestTrafficAt=0;
let latestVehicle={car:{dir:'NONE',dist:null},boxes:[],n:0,ms:0,w:640,h:480};
let latestVehicleAt=0;
let recordingState={active:false,frames:0,last_path:''};
let motorState={usb:false,stack_running:false,stack_state:'unknown',active:false,missing_processes:[],result:'잠김',error:null};
let recordDriveState={state:'idle',result:'대기 중',can_start:true,can_pause:false,can_resume:false,can_finish:false,recording:{frames:0}};
let recordDriveSettings={speed:0.5};
let recoveryState={busy:false,result:'복구 대기 중',error:null,components:{}};
let driveSettings={speed:0.5,duration:10,steering:'center'};
let driveStateText='잠김';
let motorBusy=false;
let selectedDevice=null;
let toastTimer;
let streamNonce=Date.now();
let layoutSize=localStorage.getItem('kobotViewerLayout')==='3'?3:2;
let slotTypes=loadSlots();

function loadSlots(){
  try{
    const saved=JSON.parse(localStorage.getItem('kobotViewerSlots')||'[]');
    if(saved.length&&!saved.includes('lane'))saved[1]='lane';
    return defaultSlots.map((fallback,index)=>viewLabels[saved[index]]?saved[index]:fallback);
  }catch(error){return [...defaultSlots]}
}

function toast(message){
  const el=document.querySelector('#toast');
  el.textContent=message;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>el.classList.remove('show'),2600);
}

async function api(path,options={}){
  const response=await fetch(path,{headers:{'Content-Type':'application/json'},...options});
  const body=await response.json();
  if(!response.ok)throw new Error(body.error||'요청 실패');
  return body;
}

function viewOptions(selected){
  return Object.entries(viewLabels).map(([value,label])=>`<option value="${value}" ${value===selected?'selected':''}>${label}</option>`).join('');
}

function streamUrl(topic){
  return `http://${host}:8080/stream?topic=${topic}&_=${streamNonce}`;
}

function closeStreamConnections(root=document){
  root.querySelectorAll('img[data-stream-topic]').forEach(image=>image.removeAttribute('src'));
}

function refreshStreamConnections(){
  const images=[...document.querySelectorAll('img[data-stream-topic]')];
  streamNonce=Date.now();
  images.forEach(image=>image.removeAttribute('src'));
  return new Promise(resolve=>setTimeout(()=>{
    images.forEach(image=>{if(image.isConnected)image.src=streamUrl(image.dataset.streamTopic)});
    resolve();
  },250));
}

function cameraCard(){
  return `<div class="card-heading"><div><p class="eyebrow">LIVE CAMERA</p><h3>전방 카메라</h3></div><span class="live-badge">LIVE</span></div>
    <div class="camera-frame"><img class="camera-stream" data-stream-topic="/image_raw" src="${streamUrl('/image_raw')}" alt="차량 전방 카메라 영상"></div>
    <div class="record-controls"><button class="primary record-start">저장 시작</button><button class="danger record-stop">저장 정지</button></div>
    <p class="helper record-status">저장 상태 확인 중</p>`;
}

function laneStreamTopic(){
  return '/kobot_yolop/debug_image';
}

function laneCard(){
  return `<div class="card-heading"><div><p class="eyebrow">YOLOPv2 CUDA · 신호등 색도판정 · MOTOR OFF</p><h3>차선 + 신호등 인식</h3></div><span class="live-badge">LIVE</span></div>
    <div class="camera-frame lane-frame"><img class="lane-stream" data-topic="${laneStreamTopic()}" data-stream-topic="${laneStreamTopic()}" src="${streamUrl(laneStreamTopic())}" alt="차선 인식 결과 영상"><div class="signal-overlay"></div><div class="vehicle-overlay"></div></div>
    <div class="lane-row">${signalCardHtml()}${vehicleCardHtml()}</div>
    <div class="lane-stats">${laneStatsHtml()}</div>
    <p class="helper">노랑: 중앙선 · 주황: 왼쪽 차선 · 보라: 오른쪽 차선 · 하늘색: 예정 경로 · 빨간 십자: 목표점 · 청록 테두리: 인식된 신호등 램프 · 주황 테두리: 인식된 앞차</p>`;
}

const SIGNAL_LABEL={red:'빨간불',yellow:'노란불',green:'초록불',left:'좌회전',none:'신호 없음'};
const CAR_SIDE={LEFT:'왼쪽',RIGHT:'오른쪽',NONE:'없음'};

function vehicleCardHtml(){
  return `<div class="vehicle-card" data-side="none">
    <div class="vcar"><span class="vicon">▣</span></div>
    <strong class="vname">—</strong>
  </div>`;
}

function signalCardHtml(){
  return `<div class="signal-card" data-signal="none">
    <div class="signal-lamps">
      <span class="lamp" data-lamp="red"></span>
      <span class="lamp" data-lamp="yellow"></span>
      <span class="lamp" data-lamp="left">←</span>
      <span class="lamp" data-lamp="green"></span>
    </div>
    <span class="signal-lock" hidden>인식 꺼짐</span>
  </div>`;
}

function lidarCard(){
  return `<div class="card-heading"><div><p class="eyebrow">2D POINT VIEW</p><h3>LiDAR 실시간 점 화면</h3></div><strong class="lidar-min">-</strong></div>
    <div class="lidar-frame"><canvas class="lidar-canvas" data-lidar aria-label="LiDAR 포인트 화면"></canvas><span class="front-label">차량 앞</span></div>
    <p class="helper">바깥 원 2m · 주황: 원시 후보 · 파랑/빨강: 후보 경계 · 노랑: 채택 · 청록: 유지 · 자홍 X: 거부</p>`;
}

function ultrasonicCard(){
  return `<div class="card-heading"><div><p class="eyebrow">ULTRASONIC</p><h3>차량 초음파 배치</h3></div><span class="helper">단위 cm</span></div>
    <div class="vehicle-map">
      <div class="front-arrow">↑ 앞</div>
      <div class="car-shape" aria-label="차량 위에서 본 모양"><div class="windshield"></div><div class="roof"></div><div class="rear-glass"></div><i class="wheel wheel-fl"></i><i class="wheel wheel-fr"></i><i class="wheel wheel-rl"></i><i class="wheel wheel-rr"></i></div>
      <div class="ultrasonic-marker sensor-left"><span>왼쪽</span><strong class="ultra-value" data-sensor="left">-</strong></div>
      <div class="ultrasonic-marker sensor-right"><span>오른쪽</span><strong class="ultra-value" data-sensor="right">-</strong></div>
      <div class="ultrasonic-marker sensor-rear-left"><span>좌측 뒤</span><strong class="ultra-value" data-sensor="rearLeft">-</strong></div>
      <div class="ultrasonic-marker sensor-rear"><span>뒤</span><strong class="ultra-value" data-sensor="rear">-</strong></div>
      <div class="ultrasonic-marker sensor-rear-right"><span>우측 뒤</span><strong class="ultra-value" data-sensor="rearRight">-</strong></div>
    </div>`;
}

function imuCard(){
  return `<div class="card-heading"><div><p class="eyebrow">IMU</p><h3>차량 자세</h3></div></div>
    <div class="imu-visual"><div class="imu-axis"><span class="axis-x">X</span><span class="axis-y">Y</span><span class="axis-z">Z</span></div></div>
    <div class="metric"><span>Roll · Pitch · Yaw</span><strong class="imu-value">${latestImu}</strong></div>`;
}

function systemCard(){
  return `<div class="card-heading"><div><p class="eyebrow">SYSTEM</p><h3>차량 상태</h3></div></div>
    <div class="metric"><span>모터 USB</span><strong class="system-motor">${latestMotor}</strong></div>
    <div class="metric"><span>호스트</span><strong class="system-host">${latestHost}</strong></div>
    <pre class="system-power">${latestPower}</pre>
    <p class="helper">모터는 상태만 표시하며 이 화면에서 실행하지 않습니다.</p>`;
}

function motorStackLabel(){
  if(!motorState.usb)return 'VESC USB 없음';
  if(motorState.stack_state==='running')return '모터 연결됨';
  if(motorState.stack_state==='partial')return '모터 일부 실행';
  if(motorState.stack_state==='stopped')return '모터 중지';
  return '상태 확인 중';
}

function driveCard(){
  const badgeClass=motorState.stack_state==='running'?'ok':motorState.stack_state==='partial'?'warn':'bad';
  const missing=(motorState.missing_processes||[]).join(', ');
  const detail=motorState.stack_state==='partial'&&missing?`누락: ${missing}`:'VESC 드라이버와 명령 변환 노드를 함께 관리합니다.';
  return `<div class="card-heading"><div><p class="eyebrow">MOTOR CONTROL</p><h3>주행 테스트</h3></div><span class="motor-badge ${badgeClass}">${motorStackLabel()}</span></div>
    <div class="drive-stack"><div><span>모터 스택</span><strong class="drive-stack-state">${motorStackLabel()}</strong></div><p class="drive-stack-detail">${detail}</p><button class="restart-motor" data-drive-action="restart">모터 스택 다시 실행</button></div>
    <p class="drive-range">실제 입력 범위 · 최소 <strong>0.20 m/s</strong> / 최대 <strong>4.87 m/s</strong></p>
    <div class="drive-config">
      <label>속도 (m/s)<input class="drive-speed" type="number" min="0.20" max="4.87" step="0.10" value="${driveSettings.speed.toFixed(2)}" inputmode="decimal"></label>
      <label>실행 시간 (초)<input class="drive-duration" type="number" min="0.5" max="300" step="0.5" value="${driveSettings.duration}" inputmode="decimal"></label>
      <label>조향 방식<select class="drive-steering"><option value="center" ${driveSettings.steering==='center'?'selected':''}>중앙 고정</option><option value="left" ${driveSettings.steering==='left'?'selected':''}>왼쪽 고정</option><option value="right" ${driveSettings.steering==='right'?'selected':''}>오른쪽 고정</option><option value="sweep" ${driveSettings.steering==='sweep'?'selected':''}>좌우 반복</option></select></label>
    </div>
    <div class="drive-actions"><button class="primary" data-drive-action="test">시간 주행 시작</button><button class="danger" data-drive-action="stop">즉시 정지</button></div>
    <section class="record-drive-panel" aria-label="기록 주행">
      <div><span>기록 주행 · 노란 중앙선 추종</span><strong class="record-drive-badge">${recordDriveLabel()}</strong></div>
      <label class="record-drive-speed-field">기록 주행 속도
        <select class="record-drive-speed" aria-label="기록 주행 속도">
          ${[0.2,0.3,0.4,0.5,0.7,0.9,1.1,1.3,1.6,1.9,2.16,2.5,3.0,3.5,4.0,4.5,4.87].map(speed=>`<option value="${speed.toFixed(2)}" ${recordDriveSettings.speed===speed?'selected':''}>${speed.toFixed(2)} m/s</option>`).join('')}
        </select>
      </label>
      <p class="record-drive-help">시작 전 선택 · Space 일시정지/재개 · 종료 시 별도 폴더 저장</p>
      <p class="record-drive-help">회피를 끄면 중앙선만 따라갑니다. 먼저 중앙선 추종부터 확인하세요.</p>
      <div class="record-drive-actions"><button class="primary" data-drive-action="record-start">기록 주행 시작</button><button data-drive-action="record-toggle">일시정지</button><button class="danger" data-drive-action="record-finish">기록 주행 종료</button></div>
      <p class="record-drive-state">${recordDriveState.result||'대기 중'}</p>
    </section>
    <p class="drive-state">${driveStateText}</p>`;
}

function recordDriveLabel(){
  const labels={idle:'대기',starting:'준비 중',running:'주행 중',paused:'일시정지',ending:'종료 중',finished:'종료됨',error:'오류'};
  return labels[recordDriveState.state]||recordDriveState.state;
}

function emptyCard(){
  return `<div class="empty-card"><span>+</span><strong>표시할 화면을 선택하세요</strong><p>위 선택 메뉴에서 센서를 고를 수 있습니다.</p></div>`;
}

function cardBody(type){
  if(type==='camera')return cameraCard();
  if(type==='lane')return laneCard();
  if(type==='lidar')return lidarCard();
  if(type==='ultrasonic')return ultrasonicCard();
  if(type==='imu')return imuCard();
  if(type==='system')return systemCard();
  if(type==='drive')return driveCard();
  return emptyCard();
}

function renderGrid(){
  const count=layoutSize*layoutSize;
  closeStreamConnections(grid);
  grid.className=`cctv-grid grid-${layoutSize}`;
  grid.innerHTML=Array.from({length:count},(_,index)=>{
    const type=slotTypes[index]||'empty';
    return `<article class="viewer-card viewer-card-${type}" data-slot="${index}">
      <div class="slot-bar"><span>화면 ${index+1}</span><select class="slot-picker" aria-label="화면 ${index+1} 내용 선택">${viewOptions(type)}</select></div>
      <div class="viewer-card-body">${cardBody(type)}</div>
    </article>`;
  }).join('');
  document.querySelectorAll('[data-layout]').forEach(button=>button.classList.toggle('active',Number(button.dataset.layout)===layoutSize));
  refreshDynamicValues();
}

function vehicleStale(){
  return !latestVehicleAt || (performance.now()-latestVehicleAt)>1500;
}

function renderVehicleCards(){
  const stale=vehicleStale();
  const car=(latestVehicle.car)||{dir:'NONE',dist:null};
  const side=stale?'NONE':(car.dir||'NONE');
  const dist=car.dist;
  const has=(!stale && side!=='NONE');
  const name=has
    ? `${side}${dist!=null?' · '+Number(dist).toFixed(2)+'m':''}`
    : '—';
  const go=has?(side==='LEFT'?'우측 흰선으로 회피':'좌측 흰선으로 회피'):'';
  const sub=stale
    ? '차량 노드 신호 없음 · vehicle_perception 실행 필요'
    : (has ? go : `검출 ${latestVehicle.n||0}개 · ${Number(latestVehicle.ms||0).toFixed(0)}ms`);
  document.querySelectorAll('.vehicle-card').forEach(card=>{
    card.dataset.side=has?side.toLowerCase():'none';
    const n=card.querySelector('.vname'); if(n)n.textContent=name;
    const t=card.querySelector('.vsub'); if(t)t.textContent=sub;
  });
  document.querySelectorAll('.vehicle-overlay').forEach(drawVehicleOverlay);
}

function drawVehicleOverlay(layer){
  const frame=layer.parentElement, image=frame&&frame.querySelector('img');
  if(!image){layer.innerHTML='';return;}
  const boxes=vehicleStale()?[]:(latestVehicle.boxes||[]);
  if(!boxes.length){layer.innerHTML='';return;}
  const cw=frame.clientWidth, ch=frame.clientHeight;
  const nw=latestVehicle.w||image.naturalWidth||640;
  const nh=latestVehicle.h||image.naturalHeight||480;
  if(cw<2||ch<2||nw<2||nh<2){layer.innerHTML='';return;}
  const scale=Math.min(cw/nw,ch/nh), dw=nw*scale, dh=nh*scale;
  const ox=(cw-dw)/2, oy=(ch-dh)/2;
  layer.innerHTML=boxes.map(b=>{
    const left=ox+b.x*dw, top=oy+b.y*dh;
    const width=b.w*dw, height=b.h*dh;
    const tag=`${b.ko||'차량'} ${CAR_SIDE[b.dir]||b.dir}`
      + (b.dist!=null?` ${Number(b.dist).toFixed(2)}m`:'');
    return `<i class="vbox s-${(b.dir||'none').toLowerCase()}" style="left:${left.toFixed(1)}px;top:${top.toFixed(1)}px;width:${width.toFixed(1)}px;height:${height.toFixed(1)}px"><b>${tag}</b></i>`;
  }).join('');
}

function signalStale(){
  return !latestTrafficAt || (performance.now()-latestTrafficAt)>1500;
}

function renderSignalCards(){
  const stale=signalStale();
  const state=stale?'none':(latestTraffic.state||'none');
  const label=stale?'신호 없음':(SIGNAL_LABEL[state]||state);
  const sub=stale
    ? '신호등 노드 신호 없음 · traffic.launch.py 실행 필요'
    : `램프 ${latestTraffic.lamps||0}개 · ${Number(latestTraffic.ms||0).toFixed(0)}ms · 안정 ${SIGNAL_LABEL[latestTraffic.stable]||'-'}`;
  document.querySelectorAll('.signal-card').forEach(card=>{
    card.dataset.signal=state;
    const name=card.querySelector('.signal-name');
    if(name)name.textContent=label;
    const note=card.querySelector('.signal-sub');
    if(note)note.textContent=sub;
    card.querySelectorAll('.lamp').forEach(lamp=>{
      const which=lamp.dataset.lamp;
      const on=(state==='left')?(which==='left'||which==='green'):(which===state);
      lamp.classList.toggle('on',on);
    });
  });
  const lock=Number(latestLane.signal_lock_s||0);
  document.querySelectorAll('.signal-card').forEach(card=>{
    const badge=card.querySelector('.signal-lock');
    if(badge){badge.hidden=lock<=0;badge.textContent=`인식 꺼짐 ${Math.ceil(lock)}초`}
    card.classList.toggle('locked',lock>0);
  });
  document.querySelectorAll('.signal-overlay').forEach(drawSignalOverlay);
}

function drawSignalOverlay(layer){
  const frame=layer.parentElement, image=frame&&frame.querySelector('img');
  if(!image){layer.innerHTML='';return;}
  const boxes=signalStale()?[]:(latestTraffic.boxes||[]);
  const roi=(!signalStale()&&latestTraffic.roi)?latestTraffic.roi:null;
  if(!boxes.length&&!roi){layer.innerHTML='';return;}
  // object-fit: contain 이라 실제 그려진 영역만큼 오프셋을 계산한다
  const cw=frame.clientWidth, ch=frame.clientHeight;
  const nw=latestTraffic.w||image.naturalWidth||640;
  const nh=latestTraffic.h||image.naturalHeight||480;
  if(cw<2||ch<2||nw<2||nh<2){layer.innerHTML='';return;}
  const scale=Math.min(cw/nw,ch/nh), dw=nw*scale, dh=nh*scale;
  const ox=(cw-dw)/2, oy=(ch-dh)/2;
  const roiHtml=roi?`<i class="roi-box" style="left:${(ox+roi.x*dw).toFixed(1)}px;top:${(oy+roi.y*dh).toFixed(1)}px;width:${(roi.w*dw).toFixed(1)}px;height:${(roi.h*dh).toFixed(1)}px"><b>ROI</b></i>`:'';
  layer.innerHTML=roiHtml+boxes.map(box=>{
    const pad=6;
    const left=ox+box.x*dw-pad, top=oy+box.y*dh-pad;
    const width=box.w*dw+pad*2, height=box.h*dh+pad*2;
    return `<i class="signal-box c-${box.c}" style="left:${left.toFixed(1)}px;top:${top.toFixed(1)}px;width:${width.toFixed(1)}px;height:${height.toFixed(1)}px"></i>`;
  }).join('');
}

function refreshDynamicValues(){
  renderSignalCards();
  renderVehicleCards();
  document.querySelectorAll('.imu-value').forEach(el=>el.textContent=latestImu);
  document.querySelectorAll('.system-motor').forEach(el=>el.textContent=latestMotor);
  document.querySelectorAll('.system-host').forEach(el=>el.textContent=latestHost);
  document.querySelectorAll('.system-power').forEach(el=>el.textContent=latestPower);
  document.querySelectorAll('.lane-stats').forEach(el=>el.innerHTML=laneStatsHtml());
  document.querySelectorAll('.ultra-value').forEach(el=>{
    const value=latestUltrasonic[el.dataset.sensor];
    const marker=el.closest('.ultrasonic-marker');
    el.textContent=Number.isFinite(value)?value:'-';
    marker.classList.toggle('near',Number.isFinite(value)&&value>0&&value<30);
    marker.classList.toggle('mid',Number.isFinite(value)&&value>=30&&value<70);
  });
  const valid=latestScan?.ranges?.filter(value=>Number.isFinite(value)&&value>=latestScan.range_min&&value<=latestScan.range_max)||[];
  const minText=valid.length?`최소 ${Math.min(...valid).toFixed(2)} m`:'측정 없음';
  document.querySelectorAll('.lidar-min').forEach(el=>el.textContent=minText);
  document.querySelectorAll('.record-start').forEach(el=>el.disabled=recordingState.active);
  document.querySelectorAll('.record-stop').forEach(el=>el.disabled=!recordingState.active);
  const recordText=recordingState.active?`저장 중 · ${recordingState.frames}장 · ${recordingState.path}`:recordingState.last_path?`저장 완료 · ${recordingState.frames}장 · ${recordingState.last_path}`:'저장 대기 중';
  document.querySelectorAll('.record-status').forEach(el=>el.textContent=recordText);
  refreshDriveUi();
}

function laneNumber(value,digits,unit){
  const number=Number(value);
  return Number.isFinite(number)?number.toFixed(digits)+(unit||''):'-';
}

function laneStatsHtml(){
  const lane=latestLane||{};
  const basis={yellow:'노란 중앙선',white_center:'흰선 중앙',hold:'유지',
               none:'목표 없음'}[lane.basis]||(lane.basis||'—');
  const mode=lane.mode||'—';
  const steerDir=Number(lane.steer)<0?'steer-left':Number(lane.steer)>0?'steer-right':'';
  return [
    ['상태',mode],
    ['판정 근거',basis],
    ['조향',laneNumber(lane.steer,1,' 단위'),steerDir],
    ['속도',laneNumber(lane.speed_mps,2,' m/s')],
  ].map(([label,value,cls])=>`<div><span>${label}</span><strong class="${cls||''}">${value}</strong></div>`).join('');
}

function laneDriveLineText(){
  const lines={CENTER:'중앙선 · 2차선 중앙',SHIFT_LEFT:'왼쪽 차선으로 회피 중',SHIFT_RIGHT:'오른쪽 차선으로 회피 중',RETURNING:'중앙 복귀 중'};
  const line=lines[latestLane.drive_line]||latestLane.drive_line||'-';
  const blocked=latestLane.blocked_side==='left'?'왼쪽 막힘':latestLane.blocked_side==='right'?'오른쪽 막힘':null;
  const lidar=[['왼',latestLane.lidar_left_m],['앞',latestLane.lidar_front_m],['오',latestLane.lidar_right_m]]
    .map(([name,value])=>`${name} ${Number.isFinite(value)?Number(value).toFixed(2)+'m':'-'}`).join(' · ');
  return `${line}${blocked?' · '+blocked:''} · 라이다 ${lidar}`;
}

function laneStatusText(){
  const labels={TRACKING:'추적 중',HOLD:'차선 끊김 · 이전값 유지',HOLD_LAST:'인식 없음 · 마지막값 계속 유지',WAITING:'차선 대기',ERROR:'오류'};
  const sources={YELLOW_LINE:'노란 중앙선',YELLOW_LINE_GHOST:'직전 노란선 유지',CENTER_GAP:'센터 갭',REGION_CENTER:'영역 중앙',NONE:'경로 없음'};
  const mode=latestLane.mode||latestLane.state||'WAITING';
  const offset=Number(latestLane.center_offset_px||0);
  return `${labels[mode]||mode} · ${sources[latestLane.path_source]||latestLane.path_source||'경로 없음'} · 신뢰도 ${Number(latestLane.confidence||0).toFixed(2)} · d 오차 ${offset>=0?'+':''}${offset.toFixed(0)}px`;
}

document.querySelector('.layout-switch').addEventListener('click',event=>{
  const button=event.target.closest('button[data-layout]');
  if(!button)return;
  layoutSize=Number(button.dataset.layout);
  localStorage.setItem('kobotViewerLayout',String(layoutSize));
  renderGrid();
});

grid.addEventListener('change',event=>{
  const picker=event.target.closest('.slot-picker');
  if(picker){
    const slot=Number(picker.closest('[data-slot]').dataset.slot);
    if(picker.value==='drive')slotTypes=slotTypes.map((value,index)=>index===slot?'drive':value==='drive'?'empty':value);
    else slotTypes[slot]=picker.value;
    localStorage.setItem('kobotViewerSlots',JSON.stringify(slotTypes));
    renderGrid();
    return;
  }
  const steering=event.target.closest('.drive-steering');
  if(steering)driveSettings.steering=steering.value;
  const recordSpeed=event.target.closest('.record-drive-speed');
  if(recordSpeed){recordDriveSettings.speed=Number(recordSpeed.value);refreshDriveButtons()}
});

grid.addEventListener('input',event=>{
  if(event.target.matches('.drive-speed'))driveSettings.speed=Number(event.target.value);
  if(event.target.matches('.drive-duration'))driveSettings.duration=Number(event.target.value);
  refreshDriveButtons();
});

grid.addEventListener('click',async event=>{
  const start=event.target.closest('.record-start');
  const stop=event.target.closest('.record-stop');
  if(!start&&!stop)return;
  try{
    if(start){await api('/api/recording/start',{method:'POST',body:'{}'});toast('카메라 데이터 저장을 시작했습니다')}
    if(stop){const result=await api('/api/recording/stop',{method:'POST',body:'{}'});toast(`저장을 마쳤습니다 · ${result.frames}장`)}
    await refreshRecording();
  }catch(error){toast(`저장 오류: ${error.message}`)}
});

function deviceStatus(key){
  const state=deviceApiState[key]||{};
  const usb=state.usb!==false&&Boolean(hardwareState[key]??state.usb);
  const running=Boolean(state.running);
  return {state,usb,running,connected:usb&&running};
}

function refreshDeviceNav(){
  document.querySelectorAll('.device-icon').forEach(button=>{
    const info=deviceStatus(button.dataset.device);
    button.classList.toggle('connected',info.connected);
    button.classList.toggle('disconnected',!info.connected);
    button.title=`${deviceLabels[button.dataset.device]} · ${info.connected?'연결됨':'연결 안 됨'}`;
  });
}

function renderDeviceDialog(){
  if(!selectedDevice)return;
  const {state,usb,running,connected}=deviceStatus(selectedDevice);
  const label=running?(usb?'실행 중':'USB 대기'):'중지';
  dialogContent.innerHTML=`<div class="dialog-title-row"><div class="dialog-device-icon ${connected?'connected':'disconnected'}"><span class="connection-dot"></span></div><div><p class="eyebrow">DEVICE CONTROL</p><h2 id="device-dialog-title">${deviceLabels[selectedDevice]} 연결 관리</h2></div></div>
    <div class="device-summary"><span class="device-state ${connected?'running':'stopped'}">${label}</span><strong>${usb?'USB 인식됨':'USB 없음'}${state.pid?` · PID ${state.pid}`:''}</strong></div>
    <div class="device-actions" data-device="${selectedDevice}"><button data-action="connect" ${running||!usb?'disabled':''}>연결</button><button data-action="disconnect" ${!running?'disabled':''}>연결 해제</button><button class="reconnect" data-action="reconnect" ${!usb?'disabled':''}>재연결</button></div>
    <button id="refresh-device" class="ghost wide">상태 새로고침</button>`;
}

function openDeviceDialog(key){
  selectedDevice=key;
  renderDeviceDialog();
  dialog.hidden=false;
  syncDialogBody();
  dialog.querySelector('.dialog-close').focus();
}

function closeDeviceDialog(){
  dialog.hidden=true;
  selectedDevice=null;
  syncDialogBody();
}

function syncDialogBody(){
  document.body.classList.toggle('dialog-open',!dialog.hidden||!recoveryDialog.hidden);
}

document.querySelector('.device-nav').addEventListener('click',event=>{
  const button=event.target.closest('.device-icon');
  if(button)openDeviceDialog(button.dataset.device);
});
dialog.addEventListener('click',async event=>{
  if(event.target.closest('[data-close-dialog]')){closeDeviceDialog();return}
  if(event.target.closest('#refresh-device')){await refreshDevices();renderDeviceDialog();return}
  const button=event.target.closest('button[data-action]');
  if(!button||!selectedDevice)return;
  dialogContent.querySelectorAll('button').forEach(item=>item.disabled=true);
  try{
    const result=await api(`/api/devices/${selectedDevice}/${button.dataset.action}`,{method:'POST',body:'{}'});
    toast(`${deviceLabels[selectedDevice]}: ${result.message}`);
  }catch(error){toast(`${deviceLabels[selectedDevice]} 오류: ${error.message}`)}
  finally{setTimeout(async()=>{await refreshDevices();if(selectedDevice)renderDeviceDialog()},500)}
});

function recoveryResultText(){
  if(recoveryState.error)return `실패 · ${recoveryState.error}`;
  return recoveryState.result||'복구 대기 중';
}

function renderRecoveryDialog(){
  const busy=Boolean(recoveryState.busy);
  const components=recoveryState.components||{};
  recoveryDialogContent.innerHTML=`<div class="dialog-title-row"><div class="recovery-symbol" aria-hidden="true">↻</div><div><p class="eyebrow">STREAM & SENSOR RECOVERY</p><h2 id="recovery-dialog-title">화면·센서 복구</h2></div></div>
    <section class="recovery-safe"><div><strong>웹 영상 다시 연결</strong><p>현재 페이지의 MJPEG 연결만 닫고 새로 연결합니다. ROS와 모터는 건드리지 않습니다.</p></div><button class="primary" data-recovery-action="stream">영상 새로고침</button></section>
    <div class="recovery-section-title"><strong>개별 구성요소 재시작</strong><span>차량 정지 후 사용</span></div>
    <div class="recovery-grid">${recoveryOrder.map(name=>{
      const state=components[name]||{};
      const known=Object.prototype.hasOwnProperty.call(components,name);
      return `<button data-recovery-component="${name}" ${busy?'disabled':''}><span class="recovery-dot ${state.running?'running':'stopped'}"></span><strong>${recoveryLabels[name]}</strong><small>${known?(state.running?'실행 중':'중지'):'상태 확인 전'}</small></button>`;
    }).join('')}</div>
    <section class="recovery-groups"><button data-recovery-group="media" ${busy?'disabled':''}><strong>영상 계층 전체</strong><span>카메라 + 차선 + 영상 서버</span></button><button class="danger" data-recovery-group="all" ${busy?'disabled':''}><strong>전체 센서 재시작</strong><span>모터 안전 정지 후 모든 센서</span></button></section>
    <p class="recovery-warning">실제 구성요소 재시작은 기록 주행·저장 중 거부됩니다. 모터 스택이 켜져 있으면 먼저 0속도를 보내고 종료합니다.</p>
    <div class="recovery-status"><span>${busy?'복구 중':'현재 상태'}</span><strong>${recoveryResultText()}</strong></div>`;
}

async function refreshRecovery(){
  try{
    recoveryState=await api('/api/recovery');
    if(!recoveryDialog.hidden)renderRecoveryDialog();
  }catch(error){
    recoveryState={...recoveryState,error:error.message};
    if(!recoveryDialog.hidden)renderRecoveryDialog();
  }
}

async function openRecoveryDialog(){
  renderRecoveryDialog();
  recoveryDialog.hidden=false;
  syncDialogBody();
  recoveryDialog.querySelector('.dialog-close').focus();
  await refreshRecovery();
}

function closeRecoveryDialog(){
  recoveryDialog.hidden=true;
  syncDialogBody();
}

document.querySelector('#open-recovery').addEventListener('click',openRecoveryDialog);
recoveryDialog.addEventListener('click',async event=>{
  if(event.target.closest('[data-close-recovery]')){closeRecoveryDialog();return}
  const streamButton=event.target.closest('[data-recovery-action="stream"]');
  if(streamButton){
    streamButton.disabled=true;
    await refreshStreamConnections();
    streamButton.disabled=false;
    toast('웹 영상 연결을 새로 만들었습니다');
    return;
  }

  const componentButton=event.target.closest('[data-recovery-component]');
  const groupButton=event.target.closest('[data-recovery-group]');
  if(!componentButton&&!groupButton)return;
  const component=componentButton?.dataset.recoveryComponent;
  const group=groupButton?.dataset.recoveryGroup;
  const label=component?recoveryLabels[component]:group==='media'?'영상 계층 전체':'전체 센서';
  const warning=group==='all'
    ?'모터에 0속도를 보내고 모터 스택을 종료한 뒤 전체 센서를 재시작합니다.'
    :'모터 스택이 켜져 있으면 먼저 안전 정지 후 종료합니다.';
  if(!confirm(`차량이 정지되어 있는지 확인했습니다.\n${label} 재시작을 진행할까요?\n${warning}`))return;

  recoveryState={...recoveryState,busy:true,error:null,result:`${label} 재시작 중`};
  renderRecoveryDialog();
  try{
    const endpoint=component?`/api/recovery/component/${component}`:`/api/recovery/group/${group}`;
    recoveryState=await api(endpoint,{method:'POST',body:'{}'});
    await refreshStreamConnections();
    toast(`${label} 재시작을 완료했습니다`);
  }catch(error){
    recoveryState={...recoveryState,busy:false,error:error.message,result:'복구 실패'};
    toast(`${label} 재시작 오류: ${error.message}`);
  }finally{
    await refreshRecovery();
    renderRecoveryDialog();
  }
});
addEventListener('keydown',event=>{
  if(event.key!=='Escape')return;
  if(!recoveryDialog.hidden)closeRecoveryDialog();
  else if(!dialog.hidden)closeDeviceDialog();
});

async function refreshDevices(){
  try{
    const data=await api('/api/devices');
    deviceApiState=data.devices||{};
    refreshDeviceNav();
    if(selectedDevice)renderDeviceDialog();
  }catch(error){toast(`장치 상태 오류: ${error.message}`)}
}

ros.on('connection',()=>{rosState.textContent='ROS 연결됨';rosState.className='status-pill ok'});
ros.on('close',()=>{rosState.textContent='ROS 끊김';rosState.className='status-pill bad'});
ros.on('error',()=>{rosState.textContent='ROS 오류';rosState.className='status-pill bad'});

new ROSLIB.Topic({ros,name:'/kobot/status',messageType:'std_msgs/String'}).subscribe(message=>{
  try{
    const state=JSON.parse(message.data);
    hardwareState=state.devices||{};
    const powerLine=(state.power_mode||'').split('\n').find(line=>line.trim())||'전원 모드 확인 불가';
    latestMotor=hardwareState.motor?'인식됨 · 제어 안 함':'없음';
    latestHost=state.hostname||'kobot';
    latestPower=powerLine;
    Object.entries(hardwareState).forEach(([key,value])=>{if(deviceApiState[key])deviceApiState[key].usb=value});
    refreshDeviceNav();
    refreshDynamicValues();
    if(selectedDevice)renderDeviceDialog();
  }catch(error){}
});

new ROSLIB.Topic({ros,name:'/xycar_ultrasonic',messageType:'std_msgs/Int32MultiArray'}).subscribe(message=>{
  latestUltrasonic=Object.fromEntries(Object.entries(ultraIndex).map(([key,index])=>[key,message.data[index]]));
  refreshDynamicValues();
});

new ROSLIB.Topic({ros,name:'/imu',messageType:'sensor_msgs/Imu'}).subscribe(message=>{
  const q=message.orientation;
  const roll=Math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
  const sinp=2*(q.w*q.y-q.z*q.x);
  const pitch=Math.abs(sinp)>=1?Math.sign(sinp)*Math.PI/2:Math.asin(sinp);
  const yaw=Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  latestImu=`R ${(roll*57.3).toFixed(1)}° · P ${(pitch*57.3).toFixed(1)}° · Y ${(yaw*57.3).toFixed(1)}°`;
  refreshDynamicValues();
});

new ROSLIB.Topic({ros,name:'/scan',messageType:'sensor_msgs/LaserScan'}).subscribe(message=>{latestScan=message;refreshDynamicValues()});

new ROSLIB.Topic({ros,name:'/traffic_light/state',messageType:'std_msgs/String'}).subscribe(message=>{
  try{latestTraffic=JSON.parse(message.data);latestTrafficAt=performance.now();}catch(error){return}
  renderSignalCards();
});
setInterval(renderSignalCards,400);
addEventListener('resize',renderSignalCards);

new ROSLIB.Topic({ros,name:'/vehicle/state',messageType:'std_msgs/String'}).subscribe(message=>{
  try{latestVehicle=JSON.parse(message.data);latestVehicleAt=performance.now();}catch(error){return}
  renderVehicleCards();
});
setInterval(renderVehicleCards,400);
addEventListener('resize',renderVehicleCards);

let latestCone = null;
let latestConeAt = 0;
new ROSLIB.Topic({ros,name:'/kobot_yolop/status',messageType:'std_msgs/String'}).subscribe(message=>{
  try{latestLane=JSON.parse(message.data);refreshDynamicValues()}catch(error){}
});
new ROSLIB.Topic({ros,name:'/kobot_lane_drive/cone_corridor/status',messageType:'std_msgs/String'}).subscribe(message=>{
  try{latestCone=JSON.parse(message.data);latestConeAt=performance.now();refreshDynamicValues()}catch(error){}
});

function drawOneLidar(canvas){
  const rect=canvas.getBoundingClientRect();
  if(rect.width<1||rect.height<1)return;
  const dpr=Math.min(devicePixelRatio||1,2),width=Math.max(1,Math.floor(rect.width*dpr)),height=Math.max(1,Math.floor(rect.height*dpr));
  if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}
  const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height,cx=w/2,cy=h/2,limit=2.0,scale=Math.min(w,h)*.43/limit;
  
  // 배경 및 기본 원형 그리드
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#f8fafc';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#e2e8f0';ctx.lineWidth=dpr;
  for(let meter=0.5;meter<=limit+0.01;meter+=0.5){
    ctx.beginPath();ctx.arc(cx,cy,meter*scale,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#94a3b8';ctx.font=`${9*dpr}px -apple-system, system-ui, sans-serif`;
    const label=meter===Math.floor(meter)?`${meter}m`:`${meter.toFixed(1)}m`;
    ctx.fillText(label,cx+4*dpr,cy-meter*scale+11*dpr);
  }
  // 십자 기준선
  ctx.strokeStyle='#cbd5e1';ctx.beginPath();
  ctx.moveTo(cx,cy-limit*scale);ctx.lineTo(cx,cy+limit*scale);
  ctx.moveTo(cx-limit*scale,cy);ctx.lineTo(cx+limit*scale,cy);
  ctx.stroke();

  // 1. 라바콘 탐색 ROI 영역 박스 (전방 0.10m~1.60m, 횡방향 ±1.00m)
  const roiFwdMin = 0.10, roiFwdMax = 1.60, roiLatMax = 1.00;
  const roiXLeft = cx - roiLatMax * scale;
  const roiXRight = cx + roiLatMax * scale;
  const roiYFar = cy - roiFwdMax * scale;
  const roiYNear = cy - roiFwdMin * scale;
  
  ctx.fillStyle='rgba(16, 185, 129, 0.05)';
  ctx.fillRect(roiXLeft, roiYFar, (roiLatMax * 2) * scale, (roiFwdMax - roiFwdMin) * scale);
  ctx.strokeStyle='rgba(16, 185, 129, 0.5)';
  ctx.lineWidth=1.2*dpr;
  ctx.setLineDash([4*dpr, 3*dpr]);
  ctx.strokeRect(roiXLeft, roiYFar, (roiLatMax * 2) * scale, (roiFwdMax - roiFwdMin) * scale);
  ctx.setLineDash([]);
  
  ctx.fillStyle='rgba(16, 185, 129, 0.85)';
  ctx.font=`bold ${9*dpr}px -apple-system, system-ui, sans-serif`;
  ctx.fillText('라바콘 탐색 ROI (±1.0m × 1.6m)', roiXLeft + 4*dpr, roiYFar + 12*dpr);

  const LIDAR_OFFSET_RAD = -2.1217;
  const roiPoints = [];
  const backgroundPoints = [];

  // 2. 라이다 원시 점군 수집 및 ROI 분류
  if(latestScan && Array.isArray(latestScan.ranges)){
    for(let i=0;i<latestScan.ranges.length;i++){
      const range=latestScan.ranges[i];
      if(!Number.isFinite(range)||range<latestScan.range_min||range>Math.min(latestScan.range_max,limit))continue;
      const angle=latestScan.angle_min+i*latestScan.angle_increment+LIDAR_OFFSET_RAD;
      // 차량 좌표계: x_veh (좌+ 우-), y_veh (전방+) - ROS 차량 좌표계와 100% 일치
      const x_veh = Math.sin(angle) * range;
      const y_veh = Math.cos(angle) * range;
      
      const px = cx - x_veh * scale;
      const py = cy - y_veh * scale;

      if(y_veh >= roiFwdMin && y_veh <= roiFwdMax && Math.abs(x_veh) <= roiLatMax){
        roiPoints.push({ x_veh, y_veh, px, py });
      } else {
        backgroundPoints.push({ px, py });
      }
    }
  }

  // 3. 배경 벽면/노이즈 점군 그리기 (흐린 슬레이트색)
  ctx.fillStyle='#94a3b8';
  for(const pt of backgroundPoints){
    ctx.beginPath();
    ctx.arc(pt.px, pt.py, 1.3*dpr, 0, Math.PI*2);
    ctx.fill();
  }

  // 4. ROI 내 유클리드 군집화 (12cm 임계값) -> 라바콘 vs 벽면 분리
  const clusters = [];
  const visited = new Array(roiPoints.length).fill(false);
  for(let i=0; i<roiPoints.length; i++){
    if(visited[i]) continue;
    const c = [roiPoints[i]];
    visited[i] = true;
    const queue = [roiPoints[i]];
    while(queue.length > 0){
      const curr = queue.shift();
      for(let j=0; j<roiPoints.length; j++){
        if(!visited[j]){
          const dist = Math.hypot(roiPoints[j].x_veh - curr.x_veh, roiPoints[j].y_veh - curr.y_veh);
          if(dist <= 0.12){
            visited[j] = true;
            queue.push(roiPoints[j]);
            c.push(roiPoints[j]);
          }
        }
      }
    }
    const xs = c.map(p => p.x_veh), ys = c.map(p => p.y_veh);
    const spanX = Math.max(...xs) - Math.min(...xs);
    const spanY = Math.max(...ys) - Math.min(...ys);
    const span = Math.hypot(spanX, spanY);
    const isCone = (c.length >= 3 && c.length <= 25 && span >= 0.05 && span <= 0.25);
    const meanX = xs.reduce((a,b)=>a+b,0) / c.length;
    const meanY = ys.reduce((a,b)=>a+b,0) / c.length;
    clusters.push({ points: c, isCone, span, meanX, meanY, px: cx - meanX*scale, py: cy - meanY*scale });
  }

  const leftCones = [];
  const rightCones = [];

  // 5. 클러스터 시각화 (라바콘: 주황색 + 마커 / 벽면: 연회색)
  const allCones = [];
  for(const cl of clusters){
    if(cl.isCone){
      // 라바콘 점군 (선명한 주황색)
      ctx.fillStyle='#ea580c';
      for(const pt of cl.points){
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, 2.2*dpr, 0, Math.PI*2);
        ctx.fill();
      }
      // 라바콘 중심 마커 (원형 링 + 십자)
      ctx.strokeStyle='#f97316';
      ctx.lineWidth=1.8*dpr;
      ctx.beginPath();
      ctx.arc(cl.px, cl.py, 5*dpr, 0, Math.PI*2);
      ctx.stroke();

      allCones.push(cl);
    } else {
      // 벽면 / 대형 노이즈 (연회색 표시)
      ctx.fillStyle='#cbd5e1';
      for(const pt of cl.points){
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, 1.5*dpr, 0, Math.PI*2);
        ctx.fill();
      }
    }
  }

  // 6. 가상 폴리라인 경계선 (체이닝 방식: 최대 연결 거리 0.45m 제한)
  const MAX_LINK_DIST = 0.45;   // twinmixing_center.yaml 의 cone_max_link_distance_m 과 동일해야 한다
  const MIN_BOUNDARY_CONES = 2; // twinmixing_center.yaml 의 cone_min_boundary_cones
  const ROOT_NEAR_Y = 0.75;     // twinmixing_center.yaml 의 cone_root_max_y_m

  function chainCones(rootCandidates, pool){
    if(!rootCandidates || rootCandidates.length === 0) return [];
    // 이미 반대편 벽에 배정되어 풀에서 빠진 콘은 시작점이 될 수 없다 (상호 배제 시드 누수 방지)
    const roots = rootCandidates.filter(c => pool.includes(c));
    if(roots.length === 0) return [];

    function growFrom(root){
      const chain = [root];
      let curr = root;
      const remaining = pool.filter(c => c !== root);
      while(remaining.length > 0){
        const validNext = [];
        for(const c of remaining){
          const dist = Math.hypot(c.meanX - curr.meanX, c.meanY - curr.meanY);
          if(dist <= MAX_LINK_DIST && c.meanY >= curr.meanY - 0.10){
            validNext.push({ dist, cone: c });
          }
        }
        if(validNext.length === 0) break;
        validNext.sort((a,b) =>
          a.dist - b.dist ||
          b.cone.meanY - a.cone.meanY ||
          a.cone.meanX - b.cone.meanX ||
          a.cone.meanY - b.cone.meanY
        );
        const nextCone = validNext[0].cone;
        chain.push(nextCone);
        const idx = remaining.indexOf(nextCone);
        if(idx >= 0) remaining.splice(idx, 1);
        curr = nextCone;
      }
      return chain;
    }

    function rank(chain){
      const ys = chain.map(c => c.meanY);
      return [
        chain.length,
        Math.max(...ys) - Math.min(...ys),
        Math.max(...ys),
        -Math.hypot(chain[0].meanX, chain[0].meanY),
      ];
    }

    function better(left, right){
      const a=rank(left),b=rank(right);
      for(let i=0;i<a.length;i++){
        if(a[i]!==b[i]) return a[i]>b[i];
      }
      return false;
    }

    let best=[];
    for(const root of roots){
      const candidate=growFrom(root);
      if(best.length===0 || better(candidate,best)) best=candidate;
    }
    return best;
  }

  // 차량 가까이서 시작하는 경계만 허용한다 (cone_corridor.py와 동일).
  function selectRoots(cones, leftSide){
    const side = cones.filter(c => leftSide ? c.meanX > 0.05 : c.meanX < -0.05);
    return side.filter(c => c.meanY <= ROOT_NEAR_Y);
  }

  const leftRoots = selectRoots(allCones, true);
  const rightRoots = selectRoots(allCones, false);

  const rightDist = rightRoots.length ? Math.min(...rightRoots.map(c => Math.hypot(c.meanX, c.meanY))) : Infinity;
  const leftDist = leftRoots.length ? Math.min(...leftRoots.map(c => Math.hypot(c.meanX, c.meanY))) : Infinity;

  // 나중에 체이닝하는 벽은 시작 후보도 남은 콘에서 다시 고른다 (배제 전 목록으로 고르면 중복 배정)
  let leftChain = [], rightChain = [];
  if(rightDist <= leftDist){
    rightChain = chainCones(rightRoots, allCones);
    const remForLeft = allCones.filter(c => !rightChain.includes(c));
    leftChain = chainCones(selectRoots(remForLeft, true), remForLeft);
  } else {
    leftChain = chainCones(leftRoots, allCones);
    const remForRight = allCones.filter(c => !leftChain.includes(c));
    rightChain = chainCones(selectRoots(remForRight, false), remForRight);
  }
  if(leftChain.length < MIN_BOUNDARY_CONES) leftChain=[];
  if(rightChain.length < MIN_BOUNDARY_CONES) rightChain=[];

  if(leftChain.length > 1){
    ctx.strokeStyle='#2563eb';
    ctx.lineWidth=2.2*dpr;
    ctx.beginPath();
    ctx.moveTo(leftChain[0].px, leftChain[0].py);
    for(let i=1; i<leftChain.length; i++) ctx.lineTo(leftChain[i].px, leftChain[i].py);
    ctx.stroke();
  }
  if(rightChain.length > 1){
    ctx.strokeStyle='#dc2626';
    ctx.lineWidth=2.2*dpr;
    ctx.beginPath();
    ctx.moveTo(rightChain[0].px, rightChain[0].py);
    for(let i=1; i<rightChain.length; i++) ctx.lineTo(rightChain[i].px, rightChain[i].py);
    ctx.stroke();
  }

  // 7. 실시간 라바콘 추종 목표점 및 조향선 (최신 status 토픽 반영)
  const coneStatusFresh = latestCone && (performance.now() - latestConeAt) <= 750;
  const coneStatus = coneStatusFresh ? latestCone : null;
  const pathState = coneStatus?.path_state || (coneStatus?.found ? 'TRACKING' : 'STOPPED');
  const isHoldingPath = pathState === 'HOLD_LOST' || pathState === 'HOLD_REJECTED';
  const hasAcceptedTarget = Boolean(
    coneStatus?.found &&
    Number.isFinite(coneStatus.target_x_m) &&
    Number.isFinite(coneStatus.target_y_m)
  );
  const hasRawTarget = Boolean(
    coneStatus?.raw_found &&
    Number.isFinite(coneStatus.raw_target_x_m) &&
    Number.isFinite(coneStatus.raw_target_y_m)
  );

  // 거부된 원시 후보는 자홍색 점선과 X로 표시한다. 정상 추적 때는 채택점과
  // 완전히 겹치므로 중복 표시하지 않는다.
  if(hasRawTarget && (pathState === 'HOLD_REJECTED' || !hasAcceptedTarget)){
    const rawPx = cx - coneStatus.raw_target_x_m * scale;
    const rawPy = cy - coneStatus.raw_target_y_m * scale;
    ctx.save();
    ctx.strokeStyle='rgba(217, 70, 239, 0.9)';
    ctx.lineWidth=1.8*dpr;
    ctx.setLineDash([4*dpr, 3*dpr]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.quadraticCurveTo(cx, rawPy, rawPx, rawPy);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(rawPx, rawPy, 8*dpr, 0, Math.PI*2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(rawPx-4*dpr, rawPy-4*dpr);
    ctx.lineTo(rawPx+4*dpr, rawPy+4*dpr);
    ctx.moveTo(rawPx+4*dpr, rawPy-4*dpr);
    ctx.lineTo(rawPx-4*dpr, rawPy+4*dpr);
    ctx.stroke();
    ctx.restore();
  }

  if(hasAcceptedTarget){
    const targetPx = cx - coneStatus.target_x_m * scale;
    const targetPy = cy - coneStatus.target_y_m * scale;
    const pathColor = isHoldingPath ? '#06b6d4' : '#eab308';
    const ringColor = isHoldingPath ? '#0891b2' : '#ca8a04';

    // 정상 채택은 노란 실선, 직전 경로 hold는 청록 점선이다.
    ctx.save();
    ctx.strokeStyle=pathColor;
    ctx.lineWidth=2.5*dpr;
    if(isHoldingPath) ctx.setLineDash([6*dpr, 4*dpr]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.quadraticCurveTo(cx, targetPy, targetPx, targetPy);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle=pathColor;
    ctx.beginPath();
    ctx.arc(targetPx, targetPy, 4.5*dpr, 0, Math.PI*2);
    ctx.fill();
    ctx.strokeStyle=ringColor;
    ctx.lineWidth=1.5*dpr;
    ctx.beginPath();
    ctx.arc(targetPx, targetPy, 8*dpr, 0, Math.PI*2);
    ctx.stroke();
    ctx.restore();
  }

  // 8. 차량 중심 아이콘 (삼각형)
  ctx.fillStyle='#0f172a';
  ctx.beginPath();
  ctx.moveTo(cx,cy-9*dpr);
  ctx.lineTo(cx-7*dpr,cy+8*dpr);
  ctx.lineTo(cx+7*dpr,cy+8*dpr);
  ctx.closePath();
  ctx.fill();

  // 9. 상단 실시간 주행 상태 HUD 오버레이
  ctx.fillStyle='rgba(15, 23, 42, 0.85)';
  ctx.fillRect(8*dpr, 8*dpr, w - 16*dpr, 24*dpr);
  ctx.fillStyle='#f8fafc';
  ctx.font=`bold ${10*dpr}px -apple-system, system-ui, sans-serif`;
  if(coneStatus && coneStatus.found){
    const steerDeg = coneStatus.steering_deg || 0;
    const steerStr = `${steerDeg >= 0 ? '+' : ''}${steerDeg.toFixed(1)}°`;
    const speedStr = `${(coneStatus.speed_mps || 0).toFixed(2)}m/s`;
    const hasConeCounts = Number.isFinite(coneStatus.left_cones) && Number.isFinite(coneStatus.right_cones);
    const leftCount = hasConeCounts ? coneStatus.left_cones : (coneStatus.left_points || 0);
    const rightCount = hasConeCounts ? coneStatus.right_cones : (coneStatus.right_points || 0);
    const countLabel = hasConeCounts ? '후보 경계콘' : '반사점';
    const stateLabels = {
      TRACKING: '추적',
      HOLD_LOST: '유지·미검출',
      HOLD_REJECTED: '유지·후보거부',
    };
    const stateLabel = stateLabels[pathState] || pathState;
    let guardDetail = '';
    if(pathState === 'HOLD_REJECTED' && Number.isFinite(coneStatus.target_jump_m)){
      guardDetail = `  |  Δx ${coneStatus.target_jump_m.toFixed(2)}m`;
    }else if(pathState === 'HOLD_LOST' && Number.isFinite(coneStatus.hold_age_sec)){
      guardDetail = `  |  유지 ${coneStatus.hold_age_sec.toFixed(2)}s`;
    }
    const hudText = `라바콘 [${stateLabel}]  |  조향: ${steerStr}  |  속도: ${speedStr}  |  ${countLabel} L:${leftCount} R:${rightCount}${guardDetail}`;
    ctx.fillText(hudText, 14*dpr, 24*dpr);
  } else {
    const reason = coneStatus?.rejection_reason;
    if(latestCone && !coneStatusFresh){
      ctx.fillStyle='#fca5a5';
      ctx.fillText('라바콘 [상태 끊김] · status 수신 대기', 14*dpr, 24*dpr);
      return;
    }
    ctx.fillStyle=pathState === 'STOPPED' ? '#fca5a5' : '#94a3b8';
    ctx.fillText(
      pathState === 'STOPPED'
        ? `라바콘 [정지] · ${reason === 'TARGET_JUMP' ? '목표점 급변' : '통로 미검출'}`
        : '라바콘 주행 대기 중 (통로 감지 대기)',
      14*dpr,
      24*dpr,
    );
  }
}

function drawLidar(){document.querySelectorAll('canvas[data-lidar]').forEach(drawOneLidar);requestAnimationFrame(drawLidar)}

async function refreshRecording(){
  try{recordingState=await api('/api/recording');refreshDynamicValues()}
  catch(error){document.querySelectorAll('.record-status').forEach(el=>el.textContent=`저장 상태 오류: ${error.message}`)}
}

function validSpeed(){return driveSettings.speed>=.2&&driveSettings.speed<=4.87}
function validRecordDriveSpeed(){return recordDriveSettings.speed>=.2&&recordDriveSettings.speed<=4.87}
function validDuration(){return driveSettings.duration>=.5&&driveSettings.duration<=300}
function setDriveState(message){driveStateText=message;document.querySelectorAll('.drive-state').forEach(el=>el.textContent=message)}
function refreshDriveUi(){
  const stackLabel=motorStackLabel();
  const missing=(motorState.missing_processes||[]).join(', ');
  document.querySelectorAll('.drive-stack-state').forEach(el=>el.textContent=stackLabel);
  document.querySelectorAll('.drive-stack-detail').forEach(el=>el.textContent=motorState.stack_state==='partial'&&missing?`누락: ${missing}`:'VESC 드라이버와 명령 변환 노드를 함께 관리합니다.');
  document.querySelectorAll('.motor-badge').forEach(el=>{
    el.textContent=stackLabel;
    el.className=`motor-badge ${motorState.stack_state==='running'?'ok':motorState.stack_state==='partial'?'warn':'bad'}`;
  });
  document.querySelectorAll('.drive-state').forEach(el=>el.textContent=driveStateText);
  document.querySelectorAll('.record-drive-badge').forEach(el=>{
    el.textContent=recordDriveLabel();
    el.dataset.state=recordDriveState.state;
  });
  const frames=recordDriveState.recording?.frames||0;
  const recordText=recordDriveState.error?`오류 · ${recordDriveState.error}`:`${recordDriveState.result||'대기 중'}${frames?` · ${frames}장`:''}`;
  document.querySelectorAll('.record-drive-state').forEach(el=>el.textContent=recordText);
  document.querySelectorAll('[data-drive-action="record-toggle"]').forEach(el=>el.textContent=recordDriveState.state==='paused'?'주행 재개':'일시정지');
  const recordBusy=['starting','running','paused','ending'].includes(recordDriveState.state);
  document.querySelectorAll('.record-drive-speed').forEach(el=>{
    if(recordBusy&&Number.isFinite(recordDriveState.selected_speed_mps))recordDriveSettings.speed=Number(recordDriveState.selected_speed_mps);
    el.value=recordDriveSettings.speed.toFixed(2);
    el.disabled=recordBusy;
  });
  const laneTopic=laneStreamTopic();
  document.querySelectorAll('.lane-stream').forEach(image=>{
    if(image.dataset.topic!==laneTopic){
      image.removeAttribute('src');
      image.dataset.topic=laneTopic;
      image.dataset.streamTopic=laneTopic;
      image.src=streamUrl(laneTopic);
    }
  });
  refreshDriveButtons();
}
function refreshDriveButtons(){
  const partial=motorState.stack_state==='partial';
  const recordBusy=['starting','running','paused','ending'].includes(recordDriveState.state);
  document.querySelectorAll('[data-drive-action="restart"]').forEach(button=>button.disabled=motorBusy||motorState.active||recordBusy||!motorState.usb);
  document.querySelectorAll('[data-drive-action="test"]').forEach(button=>button.disabled=motorBusy||recordBusy||partial||motorState.active||!validSpeed()||!validDuration());
  document.querySelectorAll('[data-drive-action="record-start"]').forEach(button=>button.disabled=motorBusy||motorState.active||motorState.stack_state!=='running'||!recordDriveState.can_start||!validRecordDriveSpeed());
  document.querySelectorAll('[data-drive-action="record-toggle"]').forEach(button=>button.disabled=motorBusy||!(recordDriveState.can_pause||recordDriveState.can_resume));
  document.querySelectorAll('[data-drive-action="record-finish"]').forEach(button=>button.disabled=motorBusy||!recordDriveState.can_finish);
}
async function handleDriveAction(action){
  if(action==='test'){
    if(!validSpeed()||!validDuration())return;
    motorBusy=true;setDriveState('시간 주행 준비 중');refreshDriveButtons();
    try{
      motorState=await api('/api/motor/test',{method:'POST',body:JSON.stringify({speed_mps:driveSettings.speed,duration_sec:driveSettings.duration,steering:driveSettings.steering})});
      setDriveState('시간 주행 시작');toast('설정한 시간 주행을 시작했습니다');
    }catch(error){setDriveState(`실행 오류 · ${error.message}`);toast(`주행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='record-start'){
    motorBusy=true;setDriveState('기록 주행 준비 중');refreshDriveButtons();
    try{
      recordDriveState=await api('/api/record-drive/start',{method:'POST',body:JSON.stringify({speed_mps:recordDriveSettings.speed})});
      setDriveState('차선 인식 준비 중');toast('기록 주행 준비를 시작했습니다');
    }catch(error){setDriveState(`기록 주행 오류 · ${error.message}`);toast(`기록 주행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='record-toggle'){
    const endpoint=recordDriveState.state==='paused'?'resume':'pause';
    motorBusy=true;refreshDriveButtons();
    try{
      recordDriveState=await api(`/api/record-drive/${endpoint}`,{method:'POST',body:'{}'});
      setDriveState(endpoint==='pause'?'기록 주행 일시정지':'기록 주행 재개');
      toast(endpoint==='pause'?'주행과 저장을 일시정지했습니다':'주행과 저장을 재개했습니다');
    }catch(error){toast(`기록 주행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='record-finish'){
    motorBusy=true;refreshDriveButtons();
    try{
      recordDriveState=await api('/api/record-drive/finish',{method:'POST',body:'{}'});
      setDriveState('기록 주행 종료');
      toast(`기록 주행을 종료했습니다 · ${recordDriveState.frames||recordDriveState.recording?.frames||0}장`);
    }catch(error){toast(`기록 주행 종료 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='restart'){
    motorBusy=true;setDriveState('모터 스택 다시 실행 중');refreshDriveButtons();
    try{motorState=await api('/api/motor/restart',{method:'POST',body:'{}'});setDriveState('모터 스택 다시 실행됨');toast('VESC 모터 스택을 다시 실행했습니다')}
    catch(error){setDriveState(`재실행 오류 · ${error.message}`);toast(`모터 재실행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='stop'){
    motorBusy=true;setDriveState('정지 명령 전송 중');refreshDriveButtons();
    try{motorState=await api('/api/motor/stop',{method:'POST',body:'{}'});setDriveState('정지됨');toast('모터를 정지했습니다')}
    catch(error){setDriveState(`정지 오류 · ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
  }
}

grid.addEventListener('click',event=>{
  const button=event.target.closest('[data-drive-action]');
  if(!button)return;
  if(button.dataset.driveAction==='record-finish'&&event.detail===0){toast('기록 주행 종료는 일시정지 후 마우스로 누르세요');return}
  handleDriveAction(button.dataset.driveAction);
});
addEventListener('keydown',event=>{
  if(event.target.matches('input,select,textarea'))return;
  if(event.code==='Space'){
    event.preventDefault();
    if(event.repeat)return;
    if(recordDriveState.state==='running'||recordDriveState.state==='paused')handleDriveAction('record-toggle');
  }
});
addEventListener('blur',()=>{if(recordDriveState.state==='running')handleDriveAction('record-toggle')});
addEventListener('pagehide',()=>{
  closeStreamConnections();
  if(recordDriveState.state==='running')fetch('/api/record-drive/pause',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true});
});

async function refreshMotorTest(){
  try{
    motorState=await api('/api/motor');
    if(motorState.active)setDriveState(`실행 중 · ${motorState.speed_mps.toFixed(2)} m/s · ${motorState.remaining_sec.toFixed(1)}초 남음`);
    else if(!motorBusy)setDriveState(motorState.error?`오류 · ${motorState.error}`:motorState.result||'잠김');
    refreshDriveUi();
  }catch(error){}
}

async function refreshRecordDrive(){
  try{
    if(recordDriveState.state==='starting'||recordDriveState.state==='running'){
      recordDriveState=await api('/api/record-drive/heartbeat',{method:'POST',body:'{}'});
    }else{
      recordDriveState=await api('/api/record-drive');
    }
    refreshDriveUi();
  }catch(error){}
}

renderGrid();
refreshDevices();
refreshRecording();
refreshDeviceNav();
requestAnimationFrame(drawLidar);
refreshDriveButtons();
refreshMotorTest();
refreshRecordDrive();
setInterval(refreshDevices,3000);
setInterval(refreshRecording,1000);
setInterval(refreshMotorTest,500);
setInterval(refreshRecordDrive,500);
