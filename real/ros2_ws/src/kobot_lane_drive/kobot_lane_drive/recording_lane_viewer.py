from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
import time
import types
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from .geometry import move_towards as _move_towards
from .lane_policy import EMPTY_OCCUPANCY, LanePolicyConfig, measure_lanes
from .obstacle_lidar import EMPTY_SECTORS
from .twinmixing_center import TwinMixingCenterConfig, extract_center_path, letterbox, render_overlay, restore_mask
from .yellow_center import YELLOW_SOURCE, YellowCenterConfig, YellowCenterTracker, extract_yellow_center

SESSION_NAME = re.compile(r"^[A-Za-z0-9_-]+$")
FRAME_NAME = re.compile(r"^\d{6}\.jpg$", re.IGNORECASE)
CACHE_NAME = "_twinmixing_replay"
MINIMUM_CONFIDENCE = 0.35
MAX_STEERING_RAD = 0.476

PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>TwinMixing 기록 분석</title><style>
*{box-sizing:border-box}body{margin:0;background:#0a0d12;color:#f4f7fb;font-family:system-ui,"Noto Sans KR",sans-serif}button,select,input{font:inherit}main{width:min(1500px,96vw);margin:auto;padding:18px 0 30px}header{display:flex;align-items:end;justify-content:space-between;gap:18px;border-bottom:1px solid #38414f;padding-bottom:14px}.eyebrow{margin:0 0 5px;color:#78d7ff;font-size:11px;font-weight:900;letter-spacing:.12em}h1{margin:0;font-size:25px}header p{margin:5px 0 0;color:#9da9b8;font-size:12px}.session{display:grid;gap:5px;min-width:330px}.session span{font-size:11px;font-weight:800;color:#b8c1cc}select,button{height:40px;border:1px solid #465364;border-radius:7px;background:#151b24;color:#f4f7fb;font-weight:750}select{padding:0 35px 0 11px}button{padding:0 15px;cursor:pointer}button:hover,button:focus-visible{border-color:#78d7ff;outline:none}.views{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:14px}.view{min-width:0;border:1px solid #333d4b;border-radius:10px;overflow:hidden;background:#05070a}.view h2{display:flex;justify-content:space-between;margin:0;padding:9px 12px;background:#151b24;font-size:13px}.view h2 span{font-size:10px;color:#9da9b8}.view img{display:block;width:100%;aspect-ratio:4/3;object-fit:contain;background:#000}.empty{display:grid;place-items:center;min-height:280px;color:#9da9b8}.empty[hidden]{display:none}.metrics{display:grid;grid-template-columns:1.25fr repeat(5,1fr);gap:8px;margin-top:10px}.metric{min-width:0;border:1px solid #333d4b;border-radius:8px;background:#121821;padding:9px 11px}.metric small{display:block;color:#8f9baa;font-size:10px;font-weight:800}.metric strong{display:block;margin-top:3px;font-size:15px;font-variant-numeric:tabular-nums;white-space:nowrap}.metric.state strong{font-size:18px}.tracking{color:#57e389}.hold,.hold_last{color:#ffd166}.waiting,.lost{color:#ff6474}.controls{display:grid;grid-template-columns:auto auto auto 1fr auto;align-items:center;gap:8px;margin-top:12px}.controls input{width:100%;accent-color:#59cef9}.counter{min-width:100px;text-align:right}.timeline{display:grid;grid-auto-flow:column;grid-auto-columns:1fr;height:30px;gap:1px;padding:3px;margin-top:12px;border:1px solid #333d4b;border-radius:7px;background:#121821;overflow:hidden}.tick{min-width:1px;border:0;border-radius:1px;padding:0;height:22px;background:#ff6474}.tick.tracking{background:#31c875}.tick.hold,.tick.hold_last{background:#f4bd36}.tick.current{outline:2px solid #fff;outline-offset:-2px}.legend{display:flex;gap:14px;margin:7px 2px;color:#9da9b8;font-size:11px}.legend i{display:inline-block;width:9px;height:9px;margin-right:5px;border-radius:2px}.help{color:#8f9baa;font-size:11px}@media(max-width:900px){header{align-items:stretch;flex-direction:column}.session{min-width:0}.views{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(3,1fr)}}@media(max-width:620px){.metrics{grid-template-columns:1fr 1fr}.controls{grid-template-columns:repeat(3,1fr)}.controls input,.counter{grid-column:1/4}.counter{text-align:center}}
</style></head><body><main><header><div><p class="eyebrow">REAL RECORDING · TWINMIXING CUDA REPLAY</p><h1>기록 주행 오버레이·상태 분석</h1><p>저장된 실제 카메라 프레임 재추론 · 모터 토픽 미사용</p></div><label class="session"><span>기록 세션</span><select id="session"></select></label></header><div id="empty" class="empty">분석 기록을 불러오는 중입니다.</div><section id="content" hidden><div class="views"><article class="view"><h2>원본 카메라 <span>RECORDED JPG</span></h2><img id="raw" alt="기록 원본"></article><article class="view"><h2>TwinMixing 오버레이 <span>CUDA REPLAY</span></h2><img id="overlay" alt="TwinMixing 오버레이"></article></div><section class="metrics"><div class="metric state"><small>제어 상태</small><strong id="mode">-</strong></div><div class="metric"><small>경로 기준</small><strong id="state">-</strong></div><div class="metric"><small>신뢰도</small><strong id="confidence">-</strong></div><div class="metric"><small>중앙 오차</small><strong id="offset">-</strong></div><div class="metric"><small>헤딩</small><strong id="heading">-</strong></div><div class="metric"><small>조향</small><strong id="steering">-</strong></div></section><section class="controls"><button id="previous">◀ 이전</button><button id="play">▶ 재생</button><button id="next">다음 ▶</button><input id="slider" type="range" min="0" max="0" value="0"><strong id="counter" class="counter">0 / 0</strong></section><div id="timeline" class="timeline"></div><div class="legend"><span><i style="background:#31c875"></i>TRACKING</span><span><i style="background:#f4bd36"></i>HOLD_LAST</span><span><i style="background:#ff6474"></i>초기 WAITING</span></div><p id="detail" class="help"></p></section></main><script>
const $=s=>document.querySelector(s),session=$('#session'),empty=$('#empty'),content=$('#content'),raw=$('#raw'),overlay=$('#overlay'),slider=$('#slider'),counter=$('#counter'),timeline=$('#timeline');let frames=[],index=0,timer=null;const fmt=(v,d=1,s='')=>v==null?'-':`${Number(v).toFixed(d)}${s}`;function setPlaying(on){if(timer){clearInterval(timer);timer=null}$('#play').textContent=on?'❚❚ 일시정지':'▶ 재생';if(on)timer=setInterval(()=>{if(index>=frames.length-1)return setPlaying(false);show(index+1)},250)}function show(next){if(!frames.length)return;index=Math.max(0,Math.min(frames.length-1,Number(next)||0));const x=frames[index],name=session.value;raw.src=`/api/frame?kind=raw&session=${encodeURIComponent(name)}&index=${index}`;overlay.src=`/api/frame?kind=overlay&session=${encodeURIComponent(name)}&index=${index}`;slider.value=index;counter.textContent=`${index+1} / ${frames.length}`;$('#mode').textContent=x.mode;$('#mode').className=x.mode.toLowerCase();$('#state').textContent=x.path_source;$('#confidence').textContent=fmt(x.confidence,2);$('#offset').textContent=fmt(x.center_offset_px,1,' px');$('#heading').textContent=fmt(x.heading_deg,1,'°');$('#steering').textContent=fmt(x.steering_rad,3,' rad');$('#detail').textContent=`${x.filename} · ${x.reason} · 추론 ${fmt(x.inference_ms,1,' ms')} · ${x.device}`;timeline.querySelectorAll('.current').forEach(e=>e.classList.remove('current'));timeline.children[index]?.classList.add('current')}function buildTimeline(){timeline.innerHTML='';frames.forEach((x,i)=>{const b=document.createElement('button');b.className=`tick ${x.mode.toLowerCase()}`;b.title=`${i+1}: ${x.mode} · ${x.path_source}`;b.onclick=()=>show(i);timeline.appendChild(b)})}async function selectSession(){setPlaying(false);const r=await fetch(`/api/analysis?session=${encodeURIComponent(session.value)}`,{cache:'no-store'});if(!r.ok)throw new Error(await r.text());frames=(await r.json()).frames||[];slider.max=Math.max(0,frames.length-1);buildTimeline();content.hidden=false;empty.hidden=true;show(0)}async function init(){try{const r=await fetch('/api/sessions',{cache:'no-store'}),items=(await r.json()).sessions||[];session.innerHTML=items.map(x=>`<option value="${x.name}">${x.name} · ${x.frames}장 · ${x.lost_frames} HOLD_LAST</option>`).join('');if(!items.length)throw new Error('분석 완료된 기록이 없습니다.');await selectSession()}catch(e){empty.textContent=`불러오기 실패: ${e.message}`}}$('#previous').onclick=()=>show(index-1);$('#next').onclick=()=>show(index+1);$('#play').onclick=()=>setPlaying(!timer);slider.oninput=()=>show(slider.value);session.onchange=()=>selectSession();addEventListener('keydown',e=>{if(e.target.matches('input,select'))return;if(e.key==='ArrowLeft')show(index-1);else if(e.key==='ArrowRight')show(index+1);else if(e.code==='Space'){e.preventDefault();setPlaying(!timer)}});addEventListener('pagehide',()=>setPlaying(false));init();
</script></body></html>"""


@dataclass(frozen=True)
class SessionInfo:
    name: str
    frames: int
    lost_frames: int
    started_at: str

    def as_dict(self) -> dict[str, object]:
        return self.__dict__.copy()


def _frame_paths(session_path: Path) -> list[Path]:
    return sorted(p for p in session_path.iterdir() if p.is_file() and FRAME_NAME.fullmatch(p.name) and p.stat().st_size > 0)


def resolve_session(record_root: Path, session_name: str) -> Path:
    if not SESSION_NAME.fullmatch(session_name):
        raise ValueError("잘못된 세션 이름입니다.")
    root = record_root.resolve()
    path = (root / session_name).resolve()
    if path.parent != root or not path.is_dir():
        raise FileNotFoundError("기록 세션을 찾을 수 없습니다.")
    return path


def _analysis_path(session_path: Path) -> Path:
    return session_path / CACHE_NAME / "analysis.json"


def _read_analysis(session_path: Path) -> dict[str, object]:
    path = _analysis_path(session_path)
    if not path.is_file():
        raise FileNotFoundError("아직 TwinMixing 재분석되지 않은 기록입니다.")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not isinstance(payload.get("frames"), list):
        raise ValueError("분석 파일 형식이 올바르지 않습니다.")
    return payload


def list_sessions(record_root: Path) -> list[SessionInfo]:
    if not record_root.is_dir():
        return []
    sessions = []
    for path in record_root.resolve().iterdir():
        if not path.is_dir() or not SESSION_NAME.fullmatch(path.name):
            continue
        try:
            payload = _read_analysis(path)
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        frames = payload["frames"]
        sessions.append(SessionInfo(path.name, len(frames), sum(x.get("mode") not in ("TRACKING", "HOLD") for x in frames), str(payload.get("started_at", ""))))
    return sorted(sessions, key=lambda x: x.name, reverse=True)


def _install_thop_stub() -> None:
    if "thop" not in sys.modules:
        module = types.ModuleType("thop")
        module.profile = lambda *_a, **_k: None
        sys.modules["thop"] = module


def _load_model():
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA PyTorch GPU를 사용할 수 없습니다.")
    root = Path(__file__).resolve().parent.parent / "third_party" / "TwinMixing"
    checkpoint = root / "pretrained" / "large.pth"
    _install_thop_stub()
    sys.path.insert(0, str(root))
    from models.twinmixing import TwinMixing
    model = TwinMixing(SimpleNamespace(type="large", sell=False, seda=False))
    model.load_state_dict(torch.load(checkpoint, map_location="cpu", weights_only=False), strict=True)
    model = model.cuda().eval().half()
    torch.backends.cudnn.benchmark = True
    return torch, model, torch.cuda.get_device_name(0)


def _timestamps(session_path: Path, frames: list[Path]) -> list[float]:
    values = {}
    path = session_path / "timestamps.csv"
    if path.is_file():
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    values[row["filename"]] = int(row["ros_time_ns"]) / 1e9
                except (KeyError, ValueError):
                    pass
    base = frames[0].stat().st_mtime
    return [values.get(frame.name, base + i / 8.0) for i, frame in enumerate(frames)]


def prepare_session(record_root: Path, session_name: str, torch, model, device: str) -> dict[str, object]:
    session_path = resolve_session(record_root, session_name)
    sources = _frame_paths(session_path)
    if not sources:
        raise FileNotFoundError(f"JPG 프레임 없음: {session_name}")
    overlays = session_path / CACHE_NAME / "overlays"
    overlays.mkdir(parents=True, exist_ok=True)
    stamps, config = _timestamps(session_path, sources), TwinMixingCenterConfig()
    yellow_config, policy_config = YellowCenterConfig(), LanePolicyConfig()
    tracker = YellowCenterTracker(yellow_config)
    steering = last_control = last_valid = last_valid_steering = fps = 0.0
    records = []
    for index, (path, stamp) in enumerate(zip(sources, stamps)):
        frame = cv2.imread(str(path))
        widescreen = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_LINEAR)
        padded, padding = letterbox(widescreen)
        tensor = torch.from_numpy(np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1))).cuda().half().div_(255.0).unsqueeze(0)
        torch.cuda.synchronize(); started = time.perf_counter()
        with torch.inference_mode():
            driving_logits, lane_logits = model(tensor)
        torch.cuda.synchronize(); inference_ms = (time.perf_counter() - started) * 1000.0
        instant_fps = 1000.0 / max(0.001, inference_ms); fps = instant_fps if fps <= 0 else .85 * fps + .15 * instant_fps
        size = (frame.shape[1], frame.shape[0])
        driving = restore_mask(driving_logits.argmax(1)[0].byte().cpu().numpy(), padding, size)
        lane = restore_mask(lane_logits.argmax(1)[0].byte().cpu().numpy(), padding, size)
        # 우선순위 1: 색으로 확인된 노란 중앙선. 실패하면 센터 갭 / 영역 중앙으로 내려간다.
        yellow_result, yellow_evidence = extract_yellow_center(
            frame, driving, lane, yellow_config, config
        )
        tracked = tracker.update(yellow_result, frame.shape[1])
        # 외삽본은 실측 도로가 없을 때만 쓴다. 노드와 같은 순서다.
        ghost = None
        result = None
        if tracked.found and tracked.confidence >= MINIMUM_CONFIDENCE:
            if tracked.path_source == YELLOW_SOURCE:
                result = tracked
            else:
                ghost = tracked
        if result is None:
            road = extract_center_path(driving, lane, config)
            result = road if road.found else (ghost if ghost is not None else road)
        occupancy = (
            measure_lanes(driving, result.fitted_points, policy_config)
            if result.found
            else EMPTY_OCCUPANCY
        )
        if result.found and result.confidence >= MINIMUM_CONFIDENCE:
            dt = .05 if last_control <= 0 else max(.001, min(.20, stamp - last_control)); last_control = stamp
            normalized = float(result.center_offset_px) / max(1.0, frame.shape[1] * .5)
            target = float(np.clip(-.42 * 4.0 * normalized, -MAX_STEERING_RAD, MAX_STEERING_RAD))
            steering = _move_towards(steering, steering + .35 * (target - steering), 1.20 * dt)
            reason = f"target_{result.path_source.lower()}"
            last_valid, last_valid_steering, mode = stamp, steering, "TRACKING"
        elif last_valid > 0:
            steering, mode, reason = last_valid_steering, "HOLD_LAST", "hold_last_without_timeout"
        else:
            steering, mode, reason = 0.0, "WAITING", "target_line_not_found"
        output = render_overlay(
            frame, driving, lane, result, steering, inference_ms, fps, device, mode,
            yellow_mask=yellow_evidence, occupancy=occupancy, sectors=EMPTY_SECTORS, draw_hud=True,
        )
        cv2.rectangle(output, (8, frame.shape[0] - 38), (410, frame.shape[0] - 8), (0, 0, 0), -1)
        color = (80, 220, 120) if mode == "TRACKING" else (0, 210, 255) if mode == "HOLD_LAST" else (80, 80, 255)
        cv2.putText(output, f"CONTROL {mode} | {reason}", (17, frame.shape[0] - 17), cv2.FONT_HERSHEY_SIMPLEX, .52, color, 2, cv2.LINE_AA)
        if not cv2.imwrite(str(overlays / path.name), output, [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise RuntimeError(f"오버레이 저장 실패: {path.name}")
        records.append({"index": index, "filename": path.name, "timestamp": stamp, "state": result.state, "path_source": result.path_source, "mode": mode, "confidence": round(float(result.confidence), 4), "center_offset_px": None if result.center_offset_px is None else round(float(result.center_offset_px), 2), "heading_deg": None if result.heading_deg is None else round(float(result.heading_deg), 2), "steering_rad": round(float(steering), 5), "reason": reason, "inference_ms": round(inference_ms, 2), "device": device, "ghost_frames": tracker.blind_frames, "narrow_side": occupancy.narrow_side, "left_lane_width_px": round(float(occupancy.left_width_px), 1), "right_lane_width_px": round(float(occupancy.right_width_px), 1)})
        if index % 25 == 0 or index == len(sources) - 1:
            print(f"[{session_name}] {index + 1}/{len(sources)}", flush=True)
    payload = {"session": session_name, "started_at": datetime.fromtimestamp(stamps[0]).isoformat(timespec="seconds"), "generated_at": datetime.now().isoformat(timespec="seconds"), "runtime": "pytorch_cuda_fp16", "frames": records}
    _analysis_path(session_path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


class RecordingLaneViewerHandler(BaseHTTPRequestHandler):
    record_root = Path.home() / "recordings" / "camera"
    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.address_string()} - {fmt % args}")
    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status); self.send_header("Content-Type", content_type); self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store"); self.send_header("X-Content-Type-Options", "nosniff"); self.end_headers(); self.wfile.write(body)
    def _json(self, status: int, payload: dict[str, object]) -> None:
        self._send(status, json.dumps(payload, ensure_ascii=False).encode(), "application/json; charset=utf-8")
    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/": return self._send(HTTPStatus.OK, PAGE.encode(), "text/html; charset=utf-8")
            if parsed.path == "/api/sessions": return self._json(HTTPStatus.OK, {"sessions": [x.as_dict() for x in list_sessions(self.record_root)]})
            query = parse_qs(parsed.query, keep_blank_values=True); session_path = resolve_session(self.record_root, query.get("session", [""])[0])
            if parsed.path == "/api/analysis": return self._json(HTTPStatus.OK, _read_analysis(session_path))
            if parsed.path != "/api/frame": return self._json(HTTPStatus.NOT_FOUND, {"error": "찾을 수 없습니다."})
            index = int(query.get("index", ["0"])[0]); frames = _frame_paths(session_path)
            if index < 0 or index >= len(frames): raise IndexError("프레임 번호가 범위를 벗어났습니다.")
            kind = query.get("kind", ["raw"])[0]; path = frames[index] if kind == "raw" else session_path / CACHE_NAME / "overlays" / frames[index].name
            if kind not in ("raw", "overlay") or not path.is_file(): raise FileNotFoundError("분석 프레임을 찾을 수 없습니다.")
            return self._send(HTTPStatus.OK, path.read_bytes(), "image/jpeg")
        except (ValueError, IndexError, json.JSONDecodeError) as error: self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except FileNotFoundError as error: self._json(HTTPStatus.NOT_FOUND, {"error": str(error)})
        except OSError as error: self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(error)})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TwinMixing recorded overlay viewer")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--record-root", type=Path, default=Path(os.environ.get("KOBOT_RECORDINGS_DIR", str(Path.home() / "recordings" / "camera"))))
    parser.add_argument("--prepare-session", action="append", default=[]); parser.add_argument("--prepare-only", action="store_true")
    return parser


def main(args: list[str] | None = None) -> None:
    options = build_parser().parse_args(args); root = options.record_root.resolve()
    if options.prepare_session:
        torch, model, device = _load_model()
        for name in options.prepare_session: prepare_session(root, name, torch, model, device)
        if options.prepare_only: return
    RecordingLaneViewerHandler.record_root = root
    server = ThreadingHTTPServer((options.host, options.port), RecordingLaneViewerHandler)
    print(f"TwinMixing recording viewer: http://{options.host}:{options.port}", flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


if __name__ == "__main__": main()
