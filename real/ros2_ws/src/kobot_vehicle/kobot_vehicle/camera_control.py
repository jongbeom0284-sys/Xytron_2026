import re
import subprocess
import threading


class CameraControl:
    DEVICE = '/dev/videoCAM'
    CONTROL_SPECS = {
        'brightness': {'v4l2': 'brightness', 'min': -64, 'max': 64, 'recommended': 0},
        'exposure': {'v4l2': 'exposure_time_absolute', 'min': 1, 'max': 5000, 'recommended': 40},
        'gamma': {'v4l2': 'gamma', 'min': 72, 'max': 500, 'recommended': 150},
    }

    def __init__(self, device=DEVICE, runner=None):
        self.device = device
        self._runner = runner or self._run
        self._lock = threading.Lock()

    @staticmethod
    def _run(command):
        try:
            return subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except FileNotFoundError as error:
            raise RuntimeError('v4l2-ctl을 찾을 수 없습니다') from error
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('카메라 설정 응답 시간이 초과됐습니다') from error
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or '').strip()
            raise RuntimeError(detail or '카메라 설정 명령이 실패했습니다') from error

    def _command(self, argument):
        return self._runner(['v4l2-ctl', '-d', self.device, argument])

    def _status_unlocked(self):
        v4l2_names = [spec['v4l2'] for spec in self.CONTROL_SPECS.values()]
        result = self._command('--get-ctrl=' + ','.join(v4l2_names + ['auto_exposure']))
        raw_values = {}
        for line in result.stdout.splitlines():
            match = re.match(r'^\s*([a-z0-9_]+):\s*(-?\d+)', line)
            if match:
                raw_values[match.group(1)] = int(match.group(2))

        missing = [name for name in v4l2_names if name not in raw_values]
        if missing:
            raise RuntimeError('카메라 값 읽기 실패: ' + ', '.join(missing))

        values = {
            name: raw_values[spec['v4l2']]
            for name, spec in self.CONTROL_SPECS.items()
        }
        return {
            'device': self.device,
            'manual_exposure': raw_values.get('auto_exposure') == 1,
            'values': values,
            'controls': {
                name: {
                    'min': spec['min'],
                    'max': spec['max'],
                    'recommended': spec['recommended'],
                }
                for name, spec in self.CONTROL_SPECS.items()
            },
        }

    def status(self):
        with self._lock:
            return self._status_unlocked()

    def apply(self, payload):
        if not isinstance(payload, dict) or not payload:
            raise ValueError('설정할 카메라 값을 보내세요')

        unknown = sorted(set(payload) - set(self.CONTROL_SPECS))
        if unknown:
            raise ValueError('허용되지 않은 카메라 항목: ' + ', '.join(unknown))

        normalized = {}
        for name, value in payload.items():
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{name} 값은 정수여야 합니다')
            spec = self.CONTROL_SPECS[name]
            if value < spec['min'] or value > spec['max']:
                raise ValueError(
                    f'{name} 값은 {spec["min"]}~{spec["max"]} 범위여야 합니다'
                )
            normalized[name] = value

        assignments = []
        for name, spec in self.CONTROL_SPECS.items():
            if name not in normalized:
                continue
            if name == 'exposure':
                assignments.append('auto_exposure=1')
            assignments.append(f'{spec["v4l2"]}={normalized[name]}')

        with self._lock:
            self._command('--set-ctrl=' + ','.join(assignments))
            return self._status_unlocked()
