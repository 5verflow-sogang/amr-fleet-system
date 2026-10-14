"""Discord 알림 (ROS·네트워크 없이). 웹훅은 로컬 HTTP 서버로 받아 본다."""

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.notify import ENV, build_payload, context_line, group_status, notify, pick_plot  # noqa: E402
from harness.summary import summarize  # noqa: E402


def make_run(root: Path, name: str, rmse=0.02, outcome='completed', seed=1, label=''):
    run = root / name
    (run / 'eval_ekf').mkdir(parents=True)
    (run / 'meta.json').write_text(json.dumps({
        'scenario': 'loc_basic', 'label': label, 'outcome': outcome, 'seed': seed, 'git_sha': 'abc1234',
        'host': {'hostname': 'lab', 'cores': 16}, 'real_time_factor': 1.0,
        'fault': {'delay_ms': 100.0, 'loss_pct': 5.0},
    }))
    seg = {'n': 500, 'rmse': rmse, 'mean': rmse, 'max': rmse * 2}
    (run / 'eval_ekf' / 'localization_summary.json').write_text(json.dumps(
        {'robot': 'amr_01', 'segments': {'stationary': seg, 'straight': seg, 'rotation': None}}))
    return run


def group(runs=('a',), failed=(), verdicts=()):
    return {'runs': list(runs), 'failed': list(failed),
            'metrics': {f'm{i}': {'kpi': v} for i, v in enumerate(verdicts)}}


@pytest.mark.parametrize('g, expected', [
    (group(runs=()), 'FAIL'),
    (group(verdicts=['PASS', 'FAIL']), 'FAIL'),
    (group(verdicts=['PASS', 'MARGINAL']), 'MARGINAL'),
    (group(failed=[{'run_id': 'x', 'outcome': 'timeout'}], verdicts=['PASS']), 'MARGINAL'),
    (group(verdicts=['PASS']), 'PASS'),
    (group(), 'NONE'),
])
def test_group_status(g, expected):
    assert group_status(g) == expected


def test_payload_shows_kpi_and_failures(tmp_path):
    runs = [make_run(tmp_path, f'r{i}', rmse=0.01 + 0.001 * i, seed=i + 1) for i in range(5)]
    runs.append(make_run(tmp_path, 'r_fail', outcome='timeout', seed=6))
    summary = summarize(runs)
    payload = build_payload(summary, context_line(runs))

    assert payload['content'] == '하네스 결과 · git `abc1234` · lab (16코어) · 지연 100 ms · 손실 5% · seed 1~6'
    [embed] = payload['embeds']
    # 정지 RMSE는 KPI(3 cm) 안이지만 실패한 반복이 있어 MARGINAL
    assert embed['title'] == '⚠️ loc_basic'
    assert embed['description'] == '완료 5회, 실패 1회 (r_fail: timeout)'
    names = [f['name'] for f in embed['fields']]
    assert 'loc.amr_01.eval_ekf.stationary.rmse · PASS' in names
    # KPI 없는 지표(max 등)는 넣지 않는다
    assert not any(n.endswith('.max') for n in names)


def test_payload_limits():
    summary = {f'g{i}': group(verdicts=['PASS'] * 40) for i in range(12)}
    for g in summary.values():
        for d in g['metrics'].values():
            d.update(mean=1.0, ci_low=None, ci_high=None)
    payload = build_payload(summary, 'x' * 3000)
    assert len(payload['content']) == 2000
    assert len(payload['embeds']) == 10
    assert all(len(e['fields']) == 25 for e in payload['embeds'])


class _Hook(BaseHTTPRequestHandler):
    received = []
    status = 204
    # 요청마다 차례로 쓸 응답 코드. 비면 status
    statuses = []

    def do_POST(self):
        body = self.rfile.read(int(self.headers['Content-Length']))
        _Hook.received.append((dict(self.headers), body))
        self.send_response(_Hook.statuses.pop(0) if _Hook.statuses else _Hook.status)
        self.end_headers()

    def log_message(self, *args):
        pass


@pytest.fixture
def webhook(monkeypatch):
    _Hook.received, _Hook.status, _Hook.statuses = [], 204, []
    server = HTTPServer(('127.0.0.1', 0), _Hook)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monkeypatch.setenv(ENV, f'http://127.0.0.1:{server.server_port}/hook')
    yield _Hook
    server.shutdown()


def test_notify_sends_payload_and_summary_file(tmp_path, webhook):
    runs = [make_run(tmp_path, f'r{i}', seed=i + 1) for i in range(3)]
    md = tmp_path / 'summary.md'
    md.write_text('## loc_basic\n', encoding='utf-8')
    assert notify(summarize(runs), runs, md)

    [(headers, body)] = webhook.received
    assert headers['Content-Type'].startswith('multipart/form-data; boundary=')
    assert headers['User-Agent'] == 'amr-fleet-harness'
    assert b'name="payload_json"' in body and '✅ loc_basic'.encode() in body
    assert b'name="files[0]"; filename="summary.md"' in body and b'## loc_basic' in body


def add_timeline(run: Path, done: int, total: int = 4) -> Path:
    """Fleet 실행처럼 task_outcomes.csv(완료 done/total)와 plots/timeline.png를 넣는다."""
    rows = ['task_id,request_time,first_assign_time,robots,final_state,end_time']
    rows += [f't{i},1.0,2.0,amr_01,{"COMPLETED" if i < done else ""},3.0' for i in range(total)]
    (run / 'task_outcomes.csv').write_text('\n'.join(rows) + '\n')
    (run / 'plots').mkdir()
    (run / 'plots' / 'timeline.png').write_bytes(b'\x89PNG fake ' + run.name.encode())
    return run


def test_pick_plot_lowest_completion(tmp_path):
    runs = [make_run(tmp_path, f'r{i}', seed=i + 1) for i in range(4)]
    assert pick_plot(runs) is None
    add_timeline(runs[0], done=4)
    add_timeline(runs[1], done=2)
    add_timeline(runs[2], done=2)          # 같으면 앞 실행
    plot, name, note = pick_plot(runs)
    assert (plot, name) == (runs[1] / 'plots' / 'timeline.png', 'r1')
    assert '3회 중' in note and '50%' in note


def test_payload_puts_plot_in_its_group():
    summary = {'a': group(runs=['r0']), 'b': group(runs=['r1'])}
    payload = build_payload(summary, plot=('r1', '그림: r1'))
    assert 'image' not in payload['embeds'][0]
    assert payload['embeds'][1]['image'] == {'url': 'attachment://timeline.png'}
    assert payload['embeds'][1]['description'].endswith('그림: r1')


def test_notify_attaches_worst_timeline(tmp_path, webhook):
    runs = [make_run(tmp_path, f'r{i}', seed=i + 1) for i in range(2)]
    add_timeline(runs[0], done=4)
    add_timeline(runs[1], done=1)
    md = tmp_path / 'summary.md'
    md.write_text('## loc_basic\n', encoding='utf-8')
    assert notify(summarize(runs), runs, md)

    [(_, body)] = webhook.received
    assert b'name="files[0]"; filename="summary.md"' in body
    assert b'name="files[1]"; filename="timeline.png"' in body and b'Content-Type: image/png' in body
    assert b'PNG fake r1' in body and b'PNG fake r0' not in body
    assert b'attachment://timeline.png' in body


def test_notify_resends_without_plot_when_upload_fails(tmp_path, webhook, capsys):
    runs = [make_run(tmp_path, 'r0')]
    add_timeline(runs[0], done=1)
    webhook.statuses = [413]               # 그림을 붙인 첫 요청만 거절 (크기 초과)
    assert notify(summarize(runs), runs)

    [(_, first), (_, second)] = webhook.received
    assert b'filename="timeline.png"' in first
    assert b'filename="timeline.png"' not in second and b'attachment://' not in second
    assert b'name="payload_json"' in second and b'loc_basic' in second
    assert '그림 없이 다시 보낸다' in capsys.readouterr().out


def test_notify_failure_does_not_raise(tmp_path, webhook, capsys):
    webhook.status = 500
    runs = [make_run(tmp_path, 'r0')]
    assert not notify(summarize(runs), runs)
    assert 'Discord 알림 실패' in capsys.readouterr().out


def test_notify_without_webhook_does_nothing(tmp_path, monkeypatch):
    monkeypatch.delenv(ENV, raising=False)
    runs = [make_run(tmp_path, 'r0')]
    assert not notify(summarize(runs), runs)
