import os,tempfile,unittest
from logistics_gazebo_sim.world_audit import audit_world,audit_world_directory
from logistics_gazebo_sim.worlds import OUTDOOR_LAYOUTS, render_outdoor_variant, render_outdoor_world, render_world
from logistics_gazebo_sim.outdoor_preflight import metadata_for_world
import json


class WorldAuditTest(unittest.TestCase):
    def test_protruding_roof_is_rejected(self):
        from xml.etree import ElementTree
        root=ElementTree.fromstring(render_world(0))
        roof=next(m for m in root.findall("world/model") if m.get("name").endswith("_roof"))
        pose=roof.find("pose");values=pose.text.split();values[2]=str(float(values[2])+1.);pose.text=" ".join(values)
        with tempfile.TemporaryDirectory() as directory:
            path=os.path.join(directory,"bad.world");ElementTree.ElementTree(root).write(path)
            report=audit_world(path)
            self.assertFalse(report["pass"])
            self.assertTrue(any("exceeds collision" in error for error in report["errors"]))

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

    def test_outdoor_audit_requires_current_metadata_and_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            for name,layout in OUTDOOR_LAYOUTS.items():
                path=os.path.join(directory,name+'.world')
                with open(path,'w') as stream:stream.write(render_outdoor_variant(name))
                metadata=dict(layout,**metadata_for_world(name,path))
                with open(os.path.join(directory,name+'.json'),'w') as stream:
                    json.dump(metadata,stream)
                self.assertTrue(audit_world(path)['pass'])
                metadata['recommended_cruise_altitude_m']+=1
                with open(os.path.join(directory,name+'.json'),'w') as stream:
                    json.dump(metadata,stream)
                report=audit_world(path)
                self.assertFalse(report['pass'])
                self.assertIn('outdoor metadata differs from world or layout',report['errors'])

    def test_legacy_outdoor_world_uses_its_own_road_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            canonical=os.path.join(directory,'outdoor_logistics.world')
            with open(canonical,'w') as stream:stream.write(render_outdoor_world())
            self.assertTrue(audit_world(canonical)['pass'])


if __name__=="__main__":unittest.main()
