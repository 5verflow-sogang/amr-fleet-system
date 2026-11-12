"""AI 점검 (API·영상 없이): 무엇을 보낼지 고르고 메시지를 만드는 부분."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import ai_summary  # noqa: E402


def make_run(root: Path, name: str, outcome='completed', log='', video=False):
    run = root / name
    (run / 'logs').mkdir(parents=True)
    (run / 'meta.json').write_text(json.dumps({'scenario': 'demo_s11', 'outcome': outcome, 'error': 'x'}))
    (run / 'logs' / 'stack.log').write_text(log)
    if video:
        (run / 'video').mkdir()
        (run / 'video' / 'top.mp4').write_bytes(b'')
    return run


def test_nothing_to_check_skips_api(tmp_path, capsys, monkeypatch):
    make_run(tmp_path, 'r0')
    monkeypatch.setitem(sys.modules, 'anthropic', None)   # 부르면 ImportError
    assert ai_summary.main([str(tmp_path)]) == 0
    assert '영상 0개, 실패한 반복 0개' in capsys.readouterr().out


def test_picks_failed_runs_and_videos(tmp_path):
    make_run(tmp_path, 'ok', video=True)
    make_run(tmp_path, 'bad', outcome='error')
    assert [r.name for r, _ in ai_summary.failed_runs(tmp_path)] == ['bad']
    assert [v.parent.parent.name for v in ai_summary.videos(tmp_path)] == ['ok']


def test_error_lines_keep_only_errors_once(tmp_path):
    lines = ['[INFO] ok'] + ['[ERROR] [bt_navigator]: navigate_to_pose aborted'] * 3 + \
            [f'[ERROR] line {i}' for i in range(20)]
    run = make_run(tmp_path, 'bad', outcome='error', log='\n'.join(lines))
    got = ai_summary.error_lines(run / 'logs')
    assert all('[INFO]' not in g for g in got)
    assert len(got) == 10 and got[-1] == 'stack.log: [ERROR] line 19'   # 파일마다 끝쪽 10줄


def test_flags_and_message():
    good = {'blank': False, 'robots_visible': True, 'occluded': False, 'stuck': False, 'moved': True, 'note': '정상'}
    bad = dict(good, occluded=True, robots_visible=False, note='가림')
    assert ai_summary.flags(good) == []
    assert ai_summary.flags(bad) == ['로봇 안 보임', '가림']
    msg = ai_summary.build_message('runs/demo', 'https://x', 'm', [('r/top', good), ('r/west', bad)],
                                   [('r1', 'error', '시계 없음')])
    assert '`r/top` ✅' in msg and '`r/west` ⚠️ 로봇 안 보임·가림' in msg and '`r1` (error) — 시계 없음' in msg
    assert len(ai_summary.build_message('t', '', 'm', [('v', dict(good, note='가' * 3000))], [])) <= 2000


def test_parse_json_tolerates_extra_text():
    assert ai_summary.parse_json('답: {"blank": false}')['blank'] is False
    assert ai_summary.parse_json('모름') == {'note': '모름'}


def test_drive_log_kept_and_shutdown_noise_dropped(tmp_path):
    # 11/7 demo_s11 /clock 실패: 원인은 drive.log 한 줄, stack.log 오류 줄은 거의 정리(SIGINT) 소음이었다
    run = make_run(tmp_path, 'bad', outcome='error', log='\n'.join([
        "[ERROR] [map_server-37]: process[map_server-37] failed to terminate '5' seconds after receiving 'SIGINT'",
        "[ERROR] [estop_button-45]: process has died [pid 4059, exit code -2, cmd 'x']",
        '[ERROR] [amr_05.bt_navigator]: navigate_to_pose aborted']))
    (run / 'logs' / 'drive.log').write_text('시계(/clock)가 오지 않음\n')
    assert ai_summary.error_lines(run / 'logs') == [
        'drive.log: 시계(/clock)가 오지 않음', 'stack.log: [ERROR] [amr_05.bt_navigator]: navigate_to_pose aborted']


def test_call_cap(tmp_path, monkeypatch, capsys):
    # 영상 + 실패가 상한을 넘으면 실패부터 보고 나머지는 개수만 적는다
    for i in range(3):
        make_run(tmp_path, f'v{i:02d}', video=True)
    make_run(tmp_path, 'bad', outcome='error')
    monkeypatch.setattr(ai_summary, 'MAX_CALLS', 2)
    calls = []

    class Client:
        class messages:
            @staticmethod
            def create(**kw):
                calls.append(kw)
                block = type('B', (), {'type': 'text', 'text': '{"note": "ok"}'})
                return type('R', (), {'content': [block]})

    monkeypatch.setitem(sys.modules, 'anthropic', type('M', (), {'Anthropic': Client}))
    monkeypatch.setattr(ai_summary, 'frames', lambda path: [b'jpg'])
    monkeypatch.delenv(ai_summary.ENV, raising=False)
    assert ai_summary.main([str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert len(calls) == 2 and '`bad` (error)' in out and '2개는 안 봄' in out
