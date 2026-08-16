#!/usr/bin/env python3
"""Regression checks for PF replay log provenance."""

import json
from types import SimpleNamespace

from tools.compare_pf_replay import replay_diagnostics


def log(message):
    return (0, SimpleNamespace(name='particle_filter', msg=message))


def test_current_cpp_jump_rejections_are_unobservable():
    diagnostics = replay_diagnostics([
        log('Initializing C++ RangeLib method: glt'),
        log('PF sync: diagnostic fields omitted, engine cpp'),
    ])

    assert diagnostics['logged_jump_rejections'] == 'UNOBSERVABLE'
    assert diagnostics['jump_rejection_metric'] == (
        'CURRENT_CPP_NO_COMPATIBLE_EVENT_LOG')


def test_legacy_python_jump_warning_lines_remain_counted():
    warning = 'Rejected PF scan update: correction exceeds guard'
    diagnostics = replay_diagnostics([
        log('Initializing range method: glt'),
        log(warning),
        log(warning),
    ])

    assert diagnostics['logged_jump_rejections'] == 2
    assert diagnostics['jump_rejection_metric'] == (
        'LEGACY_PYTHON_THROTTLED_LOG_LINES')
    assert diagnostics['jump_rejection_lines'] == [warning, warning]


def test_unobservable_jump_metric_is_json_serializable():
    diagnostics = replay_diagnostics([
        log('PF sync: diagnostic fields omitted, engine cpp'),
    ])

    serialized = json.loads(json.dumps({'diagnostics': diagnostics}))
    assert serialized['diagnostics']['logged_jump_rejections'] == (
        'UNOBSERVABLE')
