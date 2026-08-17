const host=location.hostname;
const ros=new ROSLIB.Ros({url:`ws://${host}:9090`});
const rosState=document.querySelector('#ros-state');
const grid=document.querySelector('#cctv-grid');
const dialog=document.querySelector('#device-dialog');
const dialogContent=document.querySelector('#device-dialog-content');
const deviceLabels={camera:'카메라',lidar:'LiDAR',ultrasonic:'초음파',imu:'IMU'};
const deviceOrder=['camera','lidar','ultrasonic','imu'];
const viewLabels={camera:'전방 카메라',lane:'차선 인식',lidar:'LiDAR 점 화면',ultrasonic:'초음파 배치',imu:'IMU 자세',system:'차량 상태',drive:'주행 테스트',empty:'비어 있음'};
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
let latestLane={state:'대기 중',confidence:0,offset_px:0};
let recordingState={active:false,frames:0,last_path:''};
let motorState={usb:false,stack_running:false,stack_state:'unknown',active:false,missing_processes:[],result:'잠김',error:null};
let recordDriveState={state:'idle',result:'대기 중',can_start:true,can_pause:false,can_resume:false,can_finish:false,recording:{frames:0}};
let driveSettings={speed:0.5,duration:10,steering:'center'};
let driveStateText='잠김';
let motorBusy=false;
let selectedDevice=null;
let toastTimer;
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

function cameraCard(){
  return `<div class="card-heading"><div><p class="eyebrow">LIVE CAMERA</p><h3>전방 카메라</h3></div><span class="live-badge">LIVE</span></div>
    <div class="camera-frame"><img class="camera-stream" src="http://${host}:8080/stream?topic=/image_raw" alt="차량 전방 카메라 영상"></div>
    <div class="record-controls"><button class="primary record-start">저장 시작</button><button class="danger record-stop">저장 정지</button></div>
    <p class="helper record-status">저장 상태 확인 중</p>`;
}

function laneStreamTopic(){
  return ['starting','running','paused'].includes(recordDriveState.state)?'/kobot_lane_drive/white_left/debug_image':'/lane_preview/image';
}

function laneCard(){
  return `<div class="card-heading"><div><p class="eyebrow">LANE PREVIEW · MOTOR OFF</p><h3>차선 인식 화면</h3></div><span class="live-badge">LIVE</span></div>
    <div class="camera-frame lane-frame"><img class="lane-stream" data-topic="${laneStreamTopic()}" src="http://${host}:8080/stream?topic=${laneStreamTopic()}" alt="차선 인식 결과 영상"></div>
    <div class="metric"><span>인식 상태</span><strong class="lane-status">${laneStatusText()}</strong></div>
    <p class="helper">노란색·파란색: 좌우 경계 · 초록색: 인식된 중앙 경로 · 빨간색: 카메라 중심</p>`;
}

function lidarCard(){
  return `<div class="card-heading"><div><p class="eyebrow">2D POINT VIEW</p><h3>LiDAR 실시간 점 화면</h3></div><strong class="lidar-min">-</strong></div>
    <div class="lidar-frame"><canvas class="lidar-canvas" data-lidar aria-label="LiDAR 포인트 화면"></canvas><span class="front-label">차량 앞</span></div>
    <p class="helper">중심이 차량이며 바깥 원은 8m입니다.</p>`;
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
    <p class="drive-range">실제 입력 범위 · 최소 <strong>0.20 m/s</strong> / 최대 <strong>2.16 m/s</strong></p>
    <div class="drive-config">
      <label>속도 (m/s)<input class="drive-speed" type="number" min="0.20" max="2.16" step="0.10" value="${driveSettings.speed.toFixed(2)}" inputmode="decimal"></label>
      <label>실행 시간 (초)<input class="drive-duration" type="number" min="0.5" max="300" step="0.5" value="${driveSettings.duration}" inputmode="decimal"></label>
      <label>조향 방식<select class="drive-steering"><option value="center" ${driveSettings.steering==='center'?'selected':''}>중앙 고정</option><option value="left" ${driveSettings.steering==='left'?'selected':''}>왼쪽 고정</option><option value="right" ${driveSettings.steering==='right'?'selected':''}>오른쪽 고정</option><option value="sweep" ${driveSettings.steering==='sweep'?'selected':''}>좌우 반복</option></select></label>
    </div>
    <div class="drive-actions"><button data-drive-action="arm">WASD 시작</button><button class="primary" data-drive-action="test">시간 주행 시작</button><button class="danger" data-drive-action="stop">즉시 정지</button></div>
    <div class="keys" aria-label="WASD 원격조종"><button data-drive-key="w">W</button><button data-drive-key="a">A</button><button data-drive-key="s">S</button><button data-drive-key="d">D</button></div>
    <section class="record-drive-panel" aria-label="기록 주행">
      <div><span>기록 주행 · 좌측 차선 거리 유지</span><strong class="record-drive-badge">${recordDriveLabel()}</strong></div>
      <p class="record-drive-help">0.20 m/s · Space 일시정지/재개 · 종료는 일시정지 후 마우스</p>
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

function refreshDynamicValues(){
  document.querySelectorAll('.imu-value').forEach(el=>el.textContent=latestImu);
  document.querySelectorAll('.system-motor').forEach(el=>el.textContent=latestMotor);
  document.querySelectorAll('.system-host').forEach(el=>el.textContent=latestHost);
  document.querySelectorAll('.system-power').forEach(el=>el.textContent=latestPower);
  document.querySelectorAll('.lane-status').forEach(el=>el.textContent=laneStatusText());
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

function laneStatusText(){
  const labels={both:'양쪽 차선',left_only:'왼쪽만',right_only:'오른쪽만',no_lane:'차선 없음'};
  return `${labels[latestLane.state]||latestLane.state} · 신뢰도 ${Number(latestLane.confidence||0).toFixed(2)} · 중심 ${Number(latestLane.offset_px||0)>=0?'+':''}${Number(latestLane.offset_px||0)}px`;
}

document.querySelector('.layout-switch').addEventListener('click',event=>{
  const button=event.target.closest('button[data-layout]');
  if(!button)return;
  layoutSize=Number(button.dataset.layout);
  if(!slotTypes.slice(0,layoutSize*layoutSize).includes('drive'))disarm();
  localStorage.setItem('kobotViewerLayout',String(layoutSize));
  renderGrid();
});

grid.addEventListener('change',event=>{
  const picker=event.target.closest('.slot-picker');
  if(picker){
    const slot=Number(picker.closest('[data-slot]').dataset.slot);
    if(slotTypes[slot]==='drive'&&picker.value!=='drive')disarm();
    if(picker.value==='drive')slotTypes=slotTypes.map((value,index)=>index===slot?'drive':value==='drive'?'empty':value);
    else slotTypes[slot]=picker.value;
    localStorage.setItem('kobotViewerSlots',JSON.stringify(slotTypes));
    renderGrid();
    return;
  }
  const steering=event.target.closest('.drive-steering');
  if(steering)driveSettings.steering=steering.value;
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
  document.body.classList.add('dialog-open');
  dialog.querySelector('.dialog-close').focus();
}

function closeDeviceDialog(){
  dialog.hidden=true;
  document.body.classList.remove('dialog-open');
  selectedDevice=null;
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
addEventListener('keydown',event=>{if(event.key==='Escape'&&!dialog.hidden)closeDeviceDialog()});

async function refreshDevices(){
  try{
    const data=await api('/api/devices');
    deviceApiState=data.devices||{};
    refreshDeviceNav();
    if(selectedDevice)renderDeviceDialog();
  }catch(error){toast(`장치 상태 오류: ${error.message}`)}
}

ros.on('connection',()=>{rosState.textContent='ROS 연결됨';rosState.className='status-pill ok'});
ros.on('close',()=>{rosState.textContent='ROS 끊김';rosState.className='status-pill bad';disarm()});
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

new ROSLIB.Topic({ros,name:'/lane_preview/status',messageType:'std_msgs/String'}).subscribe(message=>{
  try{latestLane=JSON.parse(message.data);refreshDynamicValues()}catch(error){}
});

function drawOneLidar(canvas){
  const rect=canvas.getBoundingClientRect();
  if(rect.width<1||rect.height<1)return;
  const dpr=Math.min(devicePixelRatio||1,2),width=Math.max(1,Math.floor(rect.width*dpr)),height=Math.max(1,Math.floor(rect.height*dpr));
  if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}
  const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height,cx=w/2,cy=h/2,limit=8,scale=Math.min(w,h)*.43/limit;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#f8fafc';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#dbe3ed';ctx.lineWidth=dpr;
  for(let meter=2;meter<=limit;meter+=2){ctx.beginPath();ctx.arc(cx,cy,meter*scale,0,Math.PI*2);ctx.stroke();ctx.fillStyle='#91a0b4';ctx.font=`${10*dpr}px system-ui`;ctx.fillText(`${meter}m`,cx+3,cy-meter*scale+12*dpr)}
  ctx.beginPath();ctx.moveTo(cx,cy-limit*scale);ctx.lineTo(cx,cy+limit*scale);ctx.moveTo(cx-limit*scale,cy);ctx.lineTo(cx+limit*scale,cy);ctx.stroke();
  if(latestScan){for(let i=0;i<latestScan.ranges.length;i++){const range=latestScan.ranges[i];if(!Number.isFinite(range)||range<latestScan.range_min||range>Math.min(latestScan.range_max,limit))continue;const angle=latestScan.angle_min+i*latestScan.angle_increment,x=cx+Math.sin(angle)*range*scale,y=cy-Math.cos(angle)*range*scale,hue=215-Math.min(range/limit,1)*150;ctx.fillStyle=`hsl(${hue} 78% 48%)`;ctx.beginPath();ctx.arc(x,y,Math.max(1.2*dpr,1.5),0,Math.PI*2);ctx.fill()}}
  ctx.fillStyle='#172033';ctx.beginPath();ctx.moveTo(cx,cy-8*dpr);ctx.lineTo(cx-6*dpr,cy+7*dpr);ctx.lineTo(cx+6*dpr,cy+7*dpr);ctx.closePath();ctx.fill();
}

function drawLidar(){document.querySelectorAll('canvas[data-lidar]').forEach(drawOneLidar);requestAnimationFrame(drawLidar)}

async function refreshRecording(){
  try{recordingState=await api('/api/recording');refreshDynamicValues()}
  catch(error){document.querySelectorAll('.record-status').forEach(el=>el.textContent=`저장 상태 오류: ${error.message}`)}
}

const motor=new ROSLIB.Topic({ros,name:'/xycar_motor',messageType:'std_msgs/Float32MultiArray'});
let armed=false;
const pressed=new Set();
function validSpeed(){return driveSettings.speed>=.2&&driveSettings.speed<=2.16}
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
  document.querySelectorAll('[data-drive-action="arm"]').forEach(el=>el.textContent=armed?'WASD 종료':'WASD 시작');
  document.querySelectorAll('.record-drive-badge').forEach(el=>{
    el.textContent=recordDriveLabel();
    el.dataset.state=recordDriveState.state;
  });
  const frames=recordDriveState.recording?.frames||0;
  const recordText=recordDriveState.error?`오류 · ${recordDriveState.error}`:`${recordDriveState.result||'대기 중'}${frames?` · ${frames}장`:''}`;
  document.querySelectorAll('.record-drive-state').forEach(el=>el.textContent=recordText);
  document.querySelectorAll('[data-drive-action="record-toggle"]').forEach(el=>el.textContent=recordDriveState.state==='paused'?'주행 재개':'일시정지');
  const laneTopic=laneStreamTopic();
  document.querySelectorAll('.lane-stream').forEach(image=>{
    if(image.dataset.topic!==laneTopic){image.dataset.topic=laneTopic;image.src=`http://${host}:8080/stream?topic=${laneTopic}`;}
  });
  refreshDriveButtons();
}
function refreshDriveButtons(){
  const partial=motorState.stack_state==='partial';
  const recordBusy=['starting','running','paused','ending'].includes(recordDriveState.state);
  document.querySelectorAll('[data-drive-action="restart"]').forEach(button=>button.disabled=motorBusy||motorState.active||recordBusy||!motorState.usb);
  document.querySelectorAll('[data-drive-action="arm"]').forEach(button=>button.disabled=motorBusy||recordBusy||partial||!validSpeed());
  document.querySelectorAll('[data-drive-action="test"]').forEach(button=>button.disabled=motorBusy||recordBusy||partial||motorState.active||!validSpeed()||!validDuration());
  document.querySelectorAll('[data-drive-action="record-start"]').forEach(button=>button.disabled=motorBusy||armed||motorState.active||motorState.stack_state!=='running'||!recordDriveState.can_start);
  document.querySelectorAll('[data-drive-action="record-toggle"]').forEach(button=>button.disabled=motorBusy||!(recordDriveState.can_pause||recordDriveState.can_resume));
  document.querySelectorAll('[data-drive-action="record-finish"]').forEach(button=>button.disabled=motorBusy||!recordDriveState.can_finish);
}
function publish(angle,speed){motor.publish(new ROSLIB.Message({data:[angle,speed]}));setDriveState(armed?`조향 ${angle} · 속도명령 ${speed.toFixed(2)}`:'잠김')}
function zero(){pressed.clear();publish(0,0);document.querySelectorAll('[data-drive-key]').forEach(button=>button.classList.remove('active'))}
function disarm(){if(armed)zero();armed=false;setDriveState('잠김');refreshDriveUi()}
function update(){if(!armed)return;const command=driveSettings.speed/.08,speed=pressed.has('w')?command:pressed.has('s')?-command:0,angle=pressed.has('a')?-50:pressed.has('d')?50:0;publish(angle,speed)}
setInterval(update,50);

async function handleDriveAction(action){
  if(action==='arm'){
    if(!validSpeed())return;
    if(armed){disarm();return}
    motorBusy=true;setDriveState('모터 연결 중');refreshDriveButtons();
    try{motorState=await api('/api/motor/arm',{method:'POST',body:'{}'});armed=true;setDriveState('WASD 준비됨');zero()}
    catch(error){setDriveState(`시작 오류 · ${error.message}`);toast(`모터 시작 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='test'){
    if(!validSpeed()||!validDuration())return;
    disarm();motorBusy=true;setDriveState('시간 주행 준비 중');refreshDriveButtons();
    try{
      motorState=await api('/api/motor/test',{method:'POST',body:JSON.stringify({speed_mps:driveSettings.speed,duration_sec:driveSettings.duration,steering:driveSettings.steering})});
      setDriveState('시간 주행 시작');toast('설정한 시간 주행을 시작했습니다');
    }catch(error){setDriveState(`실행 오류 · ${error.message}`);toast(`주행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='record-start'){
    disarm();motorBusy=true;setDriveState('기록 주행 준비 중');refreshDriveButtons();
    try{
      recordDriveState=await api('/api/record-drive/start',{method:'POST',body:'{}'});
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
    disarm();motorBusy=true;setDriveState('모터 스택 다시 실행 중');refreshDriveButtons();
    try{motorState=await api('/api/motor/restart',{method:'POST',body:'{}'});setDriveState('모터 스택 다시 실행됨');toast('VESC 모터 스택을 다시 실행했습니다')}
    catch(error){setDriveState(`재실행 오류 · ${error.message}`);toast(`모터 재실행 오류: ${error.message}`)}
    finally{motorBusy=false;refreshDriveUi()}
    return;
  }
  if(action==='stop'){
    zero();disarm();motorBusy=true;setDriveState('정지 명령 전송 중');refreshDriveButtons();
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
function down(key){if(!armed||!['w','a','s','d'].includes(key))return;pressed.add(key);document.querySelectorAll(`[data-drive-key="${key}"]`).forEach(button=>button.classList.add('active'));update()}
function up(key){pressed.delete(key);document.querySelectorAll(`[data-drive-key="${key}"]`).forEach(button=>button.classList.remove('active'));update()}
addEventListener('keydown',event=>{
  if(event.target.matches('input,select,textarea'))return;
  const key=event.key.toLowerCase();
  if(event.code==='Space'){
    event.preventDefault();
    if(event.repeat)return;
    if(recordDriveState.state==='running'||recordDriveState.state==='paused')handleDriveAction('record-toggle');
    else if(armed)zero();
    return;
  }
  if(!event.repeat)down(key);
});
addEventListener('keyup',event=>up(event.key.toLowerCase()));
addEventListener('blur',()=>{disarm();if(recordDriveState.state==='running')handleDriveAction('record-toggle')});
addEventListener('pagehide',()=>{
  disarm();
  if(recordDriveState.state==='running')fetch('/api/record-drive/pause',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true});
});
grid.addEventListener('pointerdown',event=>{const button=event.target.closest('[data-drive-key]');if(!button)return;event.preventDefault();button.setPointerCapture(event.pointerId);down(button.dataset.driveKey)});
grid.addEventListener('pointerup',event=>{const button=event.target.closest('[data-drive-key]');if(button)up(button.dataset.driveKey)});
grid.addEventListener('pointercancel',event=>{const button=event.target.closest('[data-drive-key]');if(button)up(button.dataset.driveKey)});

async function refreshMotorTest(){
  try{
    motorState=await api('/api/motor');
    if(motorState.active)setDriveState(`실행 중 · ${motorState.speed_mps.toFixed(2)} m/s · ${motorState.remaining_sec.toFixed(1)}초 남음`);
    else if(!armed&&!motorBusy)setDriveState(motorState.error?`오류 · ${motorState.error}`:motorState.result||'잠김');
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
