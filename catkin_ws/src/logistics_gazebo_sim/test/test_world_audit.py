import os,tempfile,unittest
from logistics_gazebo_sim.world_audit import audit_world,audit_world_directory
from logistics_gazebo_sim.worlds import render_world


class WorldAuditTest(unittest.TestCase):
    def test_generated_worlds_meet_asset_budget_and_semantics(self):
        with tempfile.TemporaryDirectory() as root:
            for scene in range(7):
                with open(os.path.join(root,"scene_{}.world".format(scene)),"w") as stream:stream.write(render_world(scene))
            report=audit_world_directory(root)
            self.assertTrue(report["pass"]);self.assertEqual(report["world_count"],7)
            self.assertTrue(all(item["model_count"]<100 for item in report["worlds"]))
    def test_missing_context_fails(self):
        with tempfile.NamedTemporaryFile(suffix=".world",mode="w",delete=False) as stream:
            stream.write('<?xml version="1.0"?><sdf version="1.6"><world name="empty"/></sdf>');path=stream.name
        try:self.assertFalse(audit_world(path)["pass"])
        finally:os.unlink(path)


if __name__=="__main__":unittest.main()
