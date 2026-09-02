import unittest
from logistics_gazebo_sim.track_metrics import summarize_recorded_rows,summarize_tracks


class TrackMetricsTest(unittest.TestCase):
    def test_empty_tracks_have_zero_metrics(self):
        self.assertEqual(summarize_tracks({})["track_count"],0)

    def test_summarizes_confidence_occlusion_and_ids(self):
        value=summarize_tracks({"obstacles":[
            {"id":"b","confidence":.6,"observed":False,"occluded_for_s":.8},
            {"id":"a","confidence":1.0,"observed":True,"occluded_for_s":0},
        ]})
        self.assertEqual(value["track_count"],2);self.assertEqual(value["observed_track_count"],1)
        self.assertEqual(value["mean_track_confidence"],.8);self.assertEqual(value["max_occlusion_s"],.8)
        self.assertEqual(value["track_ids"],'["a","b"]')

    def test_summarizes_recorded_reliability_rows(self):
        rows=[
            {"track_count":"1","observed_track_count":"1","mean_track_confidence":".8","max_occlusion_s":"0","track_ids":'["bird"]'},
            {"track_count":"1","observed_track_count":"0","mean_track_confidence":".6","max_occlusion_s":".8","track_ids":'["bird"]'},
            {"track_count":"0","observed_track_count":"0","mean_track_confidence":"0","max_occlusion_s":"0","track_ids":"[]"},
        ]
        value=summarize_recorded_rows(rows)
        self.assertEqual(value["tracked_samples"],2);self.assertEqual(value["observed_samples"],1)
        self.assertEqual(value["occluded_samples"],1);self.assertEqual(value["mean_confidence"],.7)
        self.assertEqual(value["maximum_occlusion_s"],.8);self.assertEqual(value["unique_track_ids"],["bird"])

if __name__=="__main__":unittest.main()
