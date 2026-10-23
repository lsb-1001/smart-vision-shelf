"""Independent marker registry and post-navigation identity verification."""
import json
import math
from pathlib import Path


def frame_freshness(stamp_ns, now_ns, after_ns=None, last_ns=None,
                    max_age_seconds=1.5, future_tolerance_seconds=0.1):
    """Allow a bounded /clock delivery skew, never old/pre-request/repeated frames."""
    age = (now_ns-stamp_ns)/1e9
    if stamp_ns <= 0:
        return False, age, 'missing_timestamp'
    if after_ns is not None and stamp_ns <= after_ns:
        return False, age, 'before_verification_request'
    if last_ns is not None and stamp_ns <= last_ns:
        return False, age, 'repeated_or_reversed_frame'
    if age > max_age_seconds:
        return False, age, 'stale_frame'
    if age < -future_tolerance_seconds:
        return False, age, 'clock_mismatch'
    return True, age, 'fresh_with_clock_skew' if age < 0 else 'fresh'


class Registry:
    def __init__(self, data):
        if data.get('schema_version') != 1:
            raise ValueError('Unsupported registry schema')
        self.data = data
        self.by_key, self.by_id = {}, {}
        for entry in data['locations']:
            key, marker = entry['key'], entry['marker_id']
            if type(marker) is not int or marker < 0:
                raise ValueError('marker_id must be a non-negative integer')
            if key in self.by_key or marker in self.by_id:
                raise ValueError('Duplicate key or marker ID')
            pose = entry.get('approach_pose')
            if pose is not None:
                if not data.get('map_id') or pose.get('frame_id') != 'map':
                    raise ValueError('Registered approach pose needs map_id and frame_id=map')
                for field in ('x', 'y', 'yaw'):
                    value = pose[field]
                    if type(value) not in (int, float) or not math.isfinite(value):
                        raise ValueError('Pose must contain finite x/y/yaw')
            self.by_key[key], self.by_id[marker] = entry, entry
        cfg = data['verification']
        if not (0 < cfg['hold_seconds'] < cfg['deadline_seconds']
                and 0 < cfg['max_gap_seconds'] < cfg['camera_timeout_seconds']
                < cfg['deadline_seconds'] and cfg['min_frames'] >= 2):
            raise ValueError('Invalid verification timings')

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding='utf-8')))

    def get(self, key):
        try:
            return self.by_key[key.upper()]
        except (KeyError, AttributeError):
            raise ValueError('Unknown location: ' + str(key)) from None

    def lookup(self, key, map_id=None):
        entry = self.get(key)
        result = dict(location=entry['key'], marker_id=entry['marker_id'],
                      name=entry['name'], item=entry['item'], map_id=self.data['map_id'])
        if entry['approach_pose'] is None or not self.data['map_id']:
            return dict(result, status='coordinates_unregistered', approach_pose=None)
        if map_id is not None and map_id != self.data['map_id']:
            return dict(result, status='map_mismatch', approach_pose=None)
        return dict(result, status='ready', approach_pose=dict(entry['approach_pose']))


class ArrivalVerifier:
    """No movement, AMCL initial pose, cargo completion or task-state control."""
    def __init__(self, registry):
        self.registry = registry
        self.cfg = registry.data['verification']
        self.request_id = None
        self.target = None
        self.started = None
        self.last_frame = None
        self.last_observation = None
        self.ids = []
        self.state = 'idle'
        self.active = False
        self._reset_streak()

    def _reset_streak(self):
        self.signature = None
        self.since = None
        self.frames = 0

    def begin(self, key, request_id, now, navigation_arrived=False):
        if navigation_arrived is not True:
            raise ValueError('Verification requires a navigation-arrived event')
        target = self.registry.get(key)
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError('A non-empty request_id is required')
        if self.active:
            raise ValueError('Verification already active; cancel it first')
        self.request_id, self.target = request_id, target
        self.started, self.last_frame = now, None
        self.ids = []
        self.active, self.state = True, 'waiting_fresh_frame'
        self._reset_streak()
        return self.snapshot()

    def cancel(self):
        self.active, self.state = False, 'cancelled'
        self._reset_streak()
        return self.snapshot()

    def observe(self, ids, now, fresh=True):
        ids = sorted(set(ids))
        if any(type(i) is not int or i < 0 for i in ids):
            raise ValueError('Invalid observed marker ID')
        self.ids = ids
        self.last_observation = now
        if not self.active:
            return self.snapshot()
        if now < self.started:
            return self.snapshot()
        self.tick(now)
        if not self.active:
            return self.snapshot()
        # Expired requests must never be rescued by a late frame.
        if now - self.started >= self.cfg['deadline_seconds']:
            self.active, self.state = False, 'verification_timeout'
            return self.snapshot()
        if not fresh:
            self._reset_streak()
            self.state = 'waiting_fresh_frame'
            return self.snapshot()
        if self.last_frame is not None:
            gap = now - self.last_frame
            if gap <= 0 or gap > self.cfg['max_gap_seconds']:
                self._reset_streak()
        self.last_frame = now
        if not ids:
            self.state = 'marker_not_visible'
            self._reset_streak()
            return self.snapshot()
        if len(ids) > 1:
            self.state = 'ambiguous_markers'
            self._reset_streak()
            return self.snapshot()
        marker = ids[0]
        if marker not in self.registry.by_id:
            self.state = 'unknown_marker'
            self._reset_streak()
            return self.snapshot()
        if marker != self.signature:
            self.signature, self.since, self.frames = marker, now, 0
        self.frames += 1
        correct = marker == self.target['marker_id']
        self.state = 'confirming_target' if correct else 'confirming_wrong_location'
        if (self.frames >= self.cfg['min_frames']
                and now - self.since + 1e-9 >= self.cfg['hold_seconds']):
            self.active = False
            self.state = 'verified' if correct else 'wrong_location'
        return self.snapshot()

    def tick(self, now):
        if self.active:
            reference = self.last_frame if self.last_frame is not None else self.started
            if now - reference >= self.cfg['camera_timeout_seconds']:
                self.active, self.state = False, 'camera_timeout'
                self._reset_streak()
            elif now - self.started >= self.cfg['deadline_seconds']:
                self.active, self.state = False, 'verification_timeout'
                self._reset_streak()
        return self.snapshot()

    def camera_error(self):
        if self.active:
            self.active, self.state = False, 'camera_error'
            self._reset_streak()
        return self.snapshot()

    def snapshot(self):
        return dict(
            request_id=self.request_id, state=self.state, active=self.active,
            expected_id=self.target['marker_id'] if self.target else None,
            target=self.target['key'] if self.target else None,
            observed_ids=list(self.ids),
            observed_locations=[self.registry.by_id[i]['key']
                                for i in self.ids if i in self.registry.by_id],
            registered_coordinates=bool(self.target and self.target['approach_pose']),
            meaning='marker_identity_only_not_cargo_or_mission_completion')
