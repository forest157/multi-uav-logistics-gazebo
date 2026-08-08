#!/usr/bin/env python3
import os,re,unittest,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
class V041ConfigurationTest(unittest.TestCase):
    def test_launch_exposes_and_routes_scalable_options(self):
        tree=ET.parse(os.path.join(ROOT,"launch","three_uav_mission.launch"))
        root=tree.getroot();args={node.attrib["name"]:node.attrib.get("default") for node in root.findall("arg")}
        self.assertEqual(args["vehicle_count"],"3")
        self.assertEqual(args["local_avoidance_algorithm"],"collective_offset")
        xml=ET.tostring(root,encoding="unicode")
        self.assertGreaterEqual(xml.count('value="$(arg vehicle_count)"'),5)
        self.assertIn('value="$(arg local_avoidance_algorithm)"',xml)
        self.assertIn('name="expected_vehicle_count" value="$(arg vehicle_count)"',xml)
    def test_python_and_ros_package_versions_match(self):
        package=ET.parse(os.path.join(ROOT,"package.xml")).getroot().findtext("version")
        setup=Path(ROOT,"setup.py").read_text(encoding="utf-8")
        self.assertIn("version='{}'".format(package),setup)
        self.assertEqual(package,"0.4.1")
    def test_package_readme_is_clean_utf8(self):
        raw=Path(ROOT,"README.md").read_bytes();text=raw.decode("utf-8")
        self.assertNotIn("\ufffd",text)
        self.assertNotRegex(text,r"[\x00-\x08\x0b\x0c\x0e-\x1f]")
        self.assertIn("ROS 标准规划包三维实验版",text)
if __name__=="__main__":unittest.main()
