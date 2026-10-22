"""The stuck-robot watchdog fleet_agent uses."""

from rover_fleet.progress import ProgressWatchdog


def test_twitching_in_place_counts_as_stuck():
    w = ProgressWatchdog(timeout_s=15.0, min_move_m=0.3)
    w.reset((2.36, 5.82), now=0.0)
    # rocking back and forth by a few centimeters, like robot2 at the slot
    for t, pose in [(3, (2.40, 5.80)), (6, (2.33, 5.85)), (9, (2.38, 5.79)), (12, (2.35, 5.83))]:
        assert not w.stuck(pose, now=t)
    assert w.stuck((2.39, 5.81), now=15.0)


def test_moving_resets_the_clock():
    w = ProgressWatchdog(timeout_s=15.0, min_move_m=0.3)
    w.reset((0.0, 0.0), now=0.0)
    assert not w.stuck((0.1, 0.0), now=10.0)
    assert not w.stuck((0.5, 0.0), now=14.0)      # moved 0.5 m: progress, clock restarts
    assert not w.stuck((0.6, 0.0), now=28.0)      # 14 s since then
    assert w.stuck((0.6, 0.1), now=29.0)


def test_first_call_without_reset_starts_the_clock():
    w = ProgressWatchdog(timeout_s=15.0)
    assert not w.stuck((1.0, 1.0), now=100.0)
    assert w.stuck((1.0, 1.0), now=115.0)
