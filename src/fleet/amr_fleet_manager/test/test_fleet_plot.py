"""Fleet 실행 폴더 → 타임라인 그림. 구간 나누기는 matplotlib 없이, 그림 쓰기는 matplotlib이 있을 때만 확인한다."""

import pytest

from amr_fleet_manager.fleet_plot import Segment, State, main, read_events, segments


def state(t, robot='amr_01', task_id='', mode=0, x=0.0, y=0.0):
    return State(t, robot, x, y, mode, task_id)


def test_segments_task_fault_and_silent():
    states = [
        state(0.0), state(0.5, task_id='t1'), state(1.0, task_id='t1'),
        state(1.5, task_id='t2', mode=3), state(2.0, task_id='t2', mode=3),
        state(10.0, task_id='t2'),           # 8초 끊김: 앞 구간을 닫고 silent를 넣는다
        state(10.5),
        state(0.0, robot='amr_02'), state(10.5, robot='amr_02', task_id='t3'), state(11.0, robot='amr_02'),
    ]
    assert segments(states) == [
        Segment('amr_01', 0.5, 1.5, 'task', 't1'),
        Segment('amr_01', 1.5, 2.0, 'fault'),
        Segment('amr_01', 1.5, 2.0, 'task', 't2'),
        Segment('amr_01', 2.0, 10.0, 'silent'),
        Segment('amr_01', 10.0, 10.5, 'task', 't2'),
        Segment('amr_02', 0.0, 10.5, 'silent'),
        Segment('amr_02', 10.5, 11.0, 'task', 't3'),
    ]


def test_segments_robot_that_never_comes_back():
    # amr_01이 2초에 죽고 다시 뜨지 않는다. 실행 끝(amr_02의 마지막 상태)까지 silent
    states = [state(0.0, task_id='t1'), state(2.0, task_id='t1'),
              state(0.0, robot='amr_02'), state(20.0, robot='amr_02')]
    assert [g for g in segments(states) if g.robot == 'amr_01'] == [
        Segment('amr_01', 0.0, 2.0, 'task', 't1'), Segment('amr_01', 2.0, 20.0, 'silent')]


def test_read_events(tmp_path):
    (tmp_path / 'fault_events.csv').write_text('time,event\n15.0,robot_kill:amr_03\n16.0,kill\n')
    assert read_events(tmp_path / 'fault_events.csv') == [(15.0, 'robot_kill', 'amr_03'), (16.0, 'kill', '')]
    assert read_events(tmp_path / 'none.csv') == []


def test_main_writes_timeline(tmp_path):
    pytest.importorskip('matplotlib')
    rows = ['time,robot,x,y,mode,task_id']
    for i in range(40):
        t = 100.0 + i * 0.5
        rows.append(f'{t},amr_01,{i * 0.1},0.0,1,{"t1" if i < 20 else ""}')
        if i < 10:  # amr_02는 105초에 죽는다
            rows.append(f'{t},amr_02,0.0,{i * 0.1},1,t2')
    (tmp_path / 'robot_states.csv').write_text('\n'.join(rows) + '\n')
    (tmp_path / 'fault_events.csv').write_text('time,event\n105.0,robot_kill:amr_02\n')
    (tmp_path / 'recovery.csv').write_text(
        'robot,kill_time,task_id,reassign_time,reassigned_robot,recovery_s,completed\n'
        'amr_02,105.0,t2,111.0,amr_01,6.000,1\n')
    assert main([str(tmp_path)]) == 0
    assert (tmp_path / 'plots' / 'timeline.png').stat().st_size > 1000


def test_main_without_states_writes_nothing(tmp_path):
    pytest.importorskip('matplotlib')
    assert main([str(tmp_path)]) == 0
    assert not (tmp_path / 'plots').exists()
