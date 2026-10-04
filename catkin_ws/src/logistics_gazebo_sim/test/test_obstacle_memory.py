import unittest
from logistics_gazebo_sim.dynamic_obstacles import ObstacleMemory


class ObstacleMemoryTest(unittest.TestCase):
    def bird(self):
        return dict(id="bird",position=[0,12,8],velocity=[0,-3,0],radius=.75,height=1.5,observed=True,occluded_for_s=0.)

    def test_missing_target_is_predicted_not_declared_clear(self):
        memory=ObstacleMemory();memory.update([self.bird()],10.)
        held=memory.update([],11.)[0]
        self.assertEqual(held["position"],[0,9,8]);self.assertFalse(held["observed"])
        self.assertGreater(held["radius"],.75)
        self.assertEqual(memory.update([],14.1),[])

    def test_predicted_reports_cannot_extend_observation_lifetime(self):
        memory=ObstacleMemory();bird=self.bird();bird.update(observed=False,occluded_for_s=1.5)
        memory.update([bird],11.5)
        self.assertEqual(memory.update([],14.1),[])

    def test_empty_initial_feed_and_clock_reset_do_not_create_targets(self):
        memory=ObstacleMemory();self.assertEqual(memory.update([],1.),[])
        memory.update([self.bird()],10.)
        self.assertEqual(memory.update([],1.),[])
