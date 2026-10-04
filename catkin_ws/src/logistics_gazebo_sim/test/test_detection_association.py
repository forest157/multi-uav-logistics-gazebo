import unittest
from logistics_gazebo_sim.pointcloud_perception import DetectionAssociator


class DetectionAssociationTest(unittest.TestCase):
    def test_window_confirms_noisy_linear_motion_without_lowering_speed_gate(self):
        a=DetectionAssociator()
        result=[]
        for i,noise in enumerate((0.,.15,-.12,.1,-.1,.08)):
            result=a.update([dict(position=[.6*i+noise,0,8])],1.+.2*i)
        self.assertEqual(len(result),1)
        self.assertAlmostEqual(result[0]["velocity_hint"][0],3.,delta=.3)

    def test_window_does_not_confirm_stationary_jitter_or_reversal(self):
        cases=((DetectionAssociator(),(0.,.15,-.12,.1,-.1,.08)),
               (DetectionAssociator(confirmation_hits=8),(0.,.6,1.2,1.2,.6,0.,-.6,-1.2,-1.2,-.6)))
        for a,positions in cases:
            for i,x in enumerate(positions):
                self.assertEqual(a.update([dict(position=[x,0,8])],1.+.2*i),[])

    def test_confirmed_target_survives_centroid_acceleration_noise(self):
        a=DetectionAssociator(confirmation_hits=3)
        for t,x in ((0.,0.),(.2,.6),(.4,1.2)):
            result=a.update([dict(position=[x,0,8])],t)
        identity=result[0]["id"]
        for t,x in ((.6,2.),(.8,2.4),(1.,3.)):
            result=a.update([dict(position=[x,0,8])],t)
            self.assertEqual(result[0]["id"],identity)

    def test_confirmed_track_cannot_accept_impossible_speed_or_expired_match(self):
        a=DetectionAssociator(confirmation_hits=3,maximum_track_age=.5)
        for t,x in ((0.,0.),(.2,.6),(.4,1.2)):
            result=a.update([dict(position=[x,0,8])],t)
        self.assertEqual(a.update([dict(position=[2.8,0,8])],.41),[])
        self.assertEqual(a.update([dict(position=[4.2,0,8])],1.4),[])

    def test_confirms_and_keeps_stable_id(self):
        associator=DetectionAssociator(maximum_distance=2,confirmation_hits=3,maximum_misses=1)
        self.assertEqual(associator.update([{"position":[0,0,2]}]),[])
        self.assertEqual(associator.update([{"position":[0.5,0,2]}]),[])
        confirmed=associator.update([{"position":[1.0,0,2]}])
        self.assertEqual(confirmed[0]["id"],"lidar_target_0")
        self.assertEqual(associator.update([{"position":[1.5,0,2]}])[0]["id"],"lidar_target_0")

    def test_prunes_missed_track(self):
        associator=DetectionAssociator(confirmation_hits=1,maximum_misses=1)
        self.assertEqual(associator.update([{"position":[0,0,0]}])[0]["id"],"lidar_target_0")
        associator.update([]);associator.update([])
        self.assertFalse(associator.tracks)

if __name__=="__main__":unittest.main()
