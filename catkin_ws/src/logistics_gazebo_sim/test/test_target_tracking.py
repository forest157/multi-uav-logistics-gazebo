import unittest
from logistics_gazebo_sim.target_tracking import AlphaBetaTracker,TrackingError,simulate_detections,validate_detection_payload


class TargetTrackingTest(unittest.TestCase):
    def test_detection_validation_rejects_bad_xyz(self):
        with self.assertRaises(TrackingError):validate_detection_payload({"stamp":1,"detections":[{"id":"bird","position":[1,2]}]})

    def test_range_and_seeded_dropout_are_deterministic(self):
        obstacles=[{"id":"near","position":[2,0,0],"radius":1},{"id":"far","position":[20,0,0]}]
        first=simulate_detections(obstacles,[[0,0,0]],10,0.1,0,7)
        second=simulate_detections(obstacles,[[0,0,0]],10,0.1,0,7)
        self.assertEqual(first,second);self.assertEqual([row["id"] for row in first],["near"])

    def test_tracker_estimates_velocity_and_expires(self):
        tracker=AlphaBetaTracker(alpha=1,beta=1,maximum_age=0.5)
        tracker.update(1,[{"id":"bird","position":[0,0,0]}])
        track=tracker.update(2,[{"id":"bird","position":[2,0,0]}])[0]
        self.assertAlmostEqual(track["velocity"][0],2)
        self.assertTrue(tracker.update(2.25,[])[0]["observed"] is False)
        self.assertEqual(tracker.update(3,[]),[])

if __name__=="__main__":unittest.main()
