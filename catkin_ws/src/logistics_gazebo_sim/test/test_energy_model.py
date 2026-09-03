import unittest
from logistics_gazebo_sim.energy_model import EnergyEstimator,EnergyModel,mission_energy_forecast

class EnergyModelTest(unittest.TestCase):
    def test_hover_is_baseline_and_payload_costs_more(self):
        model=EnergyModel();self.assertEqual(model.power(),180.0)
        self.assertGreater(model.power(payload_kg=.5),model.power())
    def test_horizontal_climb_descent_acceleration_and_turn_are_counted(self):
        model=EnergyModel()
        base=model.power();self.assertGreater(model.power((2,0,0)),base)
        self.assertGreater(model.power((0,0,1)),model.power((0,0,-1)))
        self.assertGreater(model.power(acceleration=(1,0,0)),base)
        self.assertGreater(model.power(turn_rate=.5),base)
    def test_estimator_integrates_and_preserves_reserve(self):
        estimator=EnergyEstimator(EnergyModel(),capacity_wh=200,reserve_fraction=.2)
        estimator.update(0,(0,0,0));value=estimator.update(3600,(0,0,0))
        self.assertAlmostEqual(value["used_wh"],180);self.assertAlmostEqual(value["remaining_wh"],20)
        self.assertAlmostEqual(value["usable_margin_wh"],-20)
    def test_capacity_differences_produce_different_margin(self):
        a=EnergyEstimator(capacity_wh=220);b=EnergyEstimator(capacity_wh=180)
        a.update(0,(0,0,0));b.update(0,(0,0,0));a.update(60,(0,0,0));b.update(60,(0,0,0))
        self.assertGreater(a.snapshot()["usable_margin_wh"],b.snapshot()["usable_margin_wh"])
    def test_forecast_reports_return_wait_and_landing(self):
        value=mission_energy_forecast(EnergyModel(),120,80,30,8,payload_kg=.5)
        self.assertGreater(value["task_remaining_wh"],value["return_wh"])
        self.assertAlmostEqual(value["required_to_land_wh"],value["return_wh"]+value["wait_wh"]+value["landing_wh"],places=3)
        complete=mission_energy_forecast(EnergyModel(),0,0,0,0)
        self.assertEqual(complete["required_to_land_wh"],0.0)
    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):EnergyEstimator(capacity_wh=0)
        with self.assertRaises(ValueError):EnergyModel(hover_power_w=-1)
        with self.assertRaises(ValueError):EnergyModel().power((1,2))

if __name__=="__main__":unittest.main()
