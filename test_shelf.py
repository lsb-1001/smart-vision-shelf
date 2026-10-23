import copy
import json
from pathlib import Path
import unittest

from shelf_core import Registry, ArrivalVerifier, frame_freshness


class ShelfTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(Path(__file__).with_name('locations.json').read_text())
        self.registry = Registry(copy.deepcopy(self.data))
        self.verifier = ArrivalVerifier(self.registry)

    def begin(self, location='B', now=0):
        return self.verifier.begin(location,'request-1',now,navigation_arrived=True)

    def hold(self, ids, start=0.1):
        for i in range(5):
            result = self.verifier.observe(ids,start+i*0.2)
        return result

    def test_scenario_mapping(self):
        self.assertEqual([self.registry.get(k)['marker_id'] for k in ['A','B','C','DELIVERY']], [1,2,3,4])

    def test_missing_coordinates_not_origin(self):
        result = self.registry.lookup('B')
        self.assertEqual(result['status'],'coordinates_unregistered')
        self.assertIsNone(result['approach_pose'])

    def test_registered_map_and_pose(self):
        self.data['map_id'] = 'test-map'
        self.data['locations'][1]['approach_pose'] = dict(frame_id='map',x=1,y=2,yaw=0.1)
        registry = Registry(self.data)
        self.assertEqual(registry.lookup('B','test-map')['status'],'ready')
        self.assertIsNone(registry.lookup('B','other-map')['approach_pose'])

    def test_no_navigation_arrival(self):
        with self.assertRaises(ValueError):
            self.verifier.begin('B','test',0)

    def test_correct_id(self):
        self.begin()
        self.assertEqual(self.hold([2])['state'],'verified')

    def test_wrong_id(self):
        self.begin()
        self.assertEqual(self.hold([1])['state'],'wrong_location')

    def test_unknown_id(self):
        self.begin()
        self.assertEqual(self.hold([49])['state'],'unknown_marker')

    def test_multiple_including_target_is_ambiguous(self):
        self.begin()
        self.assertEqual(self.hold([1,2])['state'],'ambiguous_markers')

    def test_no_marker_not_success(self):
        self.begin()
        self.assertEqual(self.hold([])['state'],'marker_not_visible')

    def test_seen_before_request_does_not_verify(self):
        self.verifier.observe([2],0)
        self.begin(now=1)
        self.assertEqual(self.verifier.state,'waiting_fresh_frame')
        self.assertNotEqual(self.verifier.observe([2],1.1)['state'],'verified')

    def test_old_camera_frames_do_not_verify(self):
        self.begin()
        for t in [0.1,0.3,0.5,0.7]:
            self.assertEqual(self.verifier.observe([2],t,fresh=False)['state'],'waiting_fresh_frame')

    def test_gap_restarts_confirmation(self):
        self.begin()
        self.verifier.observe([2],0.1)
        self.verifier.observe([2],0.3)
        self.assertNotEqual(self.verifier.observe([2],0.9)['state'],'verified')

    def test_flicker_not_confirmed(self):
        self.begin()
        for i in range(6):
            self.assertTrue(self.verifier.observe([1 if i%2 else 2],0.1+i*0.2)['active'])

    def test_camera_timeout(self):
        self.begin()
        self.assertEqual(self.verifier.tick(1.6)['state'],'camera_timeout')

    def test_late_frame_cannot_rescue_camera_timeout(self):
        self.begin()
        self.assertEqual(self.verifier.observe([2],2)['state'],'camera_timeout')

    def test_verification_deadline_with_no_marker(self):
        self.begin()
        for i in range(1,50):
            self.verifier.observe([],i*0.2)
        self.assertEqual(self.verifier.tick(10.1)['state'],'verification_timeout')

    def test_new_request_resets_previous_success(self):
        self.begin()
        self.hold([2])
        self.verifier.begin('DELIVERY','request-2',2,navigation_arrived=True)
        self.assertEqual(self.verifier.state,'waiting_fresh_frame')
        self.assertEqual(self.hold([4],2.1)['state'],'verified')

    def test_cannot_replace_active_request(self):
        self.begin()
        with self.assertRaises(ValueError):
            self.verifier.begin('A','request-2',0.1,navigation_arrived=True)

    def test_invalid_coordinate(self):
        self.data['map_id'] = 'test-map'
        self.data['locations'][0]['approach_pose'] = dict(frame_id='map',x=float('nan'),y=0,yaw=0)
        with self.assertRaises(ValueError):
            Registry(self.data)

    def test_small_future_clock_delivery_skew_is_accepted(self):
        fresh,age,reason = frame_freshness(1055000000,1000000000)
        self.assertTrue(fresh)
        self.assertAlmostEqual(age,-0.055)
        self.assertEqual(reason,'fresh_with_clock_skew')

    def test_large_future_clock_mismatch_is_rejected(self):
        self.assertFalse(frame_freshness(1200000000,1000000000)[0])

    def test_old_image_is_still_rejected(self):
        self.assertFalse(frame_freshness(1000000000,3000000000)[0])

    def test_pre_request_image_is_still_rejected(self):
        self.assertFalse(frame_freshness(1055000000,1000000000,after_ns=1060000000)[0])

    def test_repeated_image_is_still_rejected(self):
        self.assertFalse(frame_freshness(1055000000,1000000000,last_ns=1055000000)[0])

    def test_missing_timestamp_is_rejected(self):
        self.assertFalse(frame_freshness(0,1000000000)[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
