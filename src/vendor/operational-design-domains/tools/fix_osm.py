# Copyright (c) 2020 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as ET

# parsing arguments
parser = argparse.ArgumentParser(description="misc fix for Scenario Editor")
parser.add_argument('-i', '--input', help="input osm map file", required=True)
parser.add_argument('-o', '--output', help="output osm map file", required=True)
parser.add_argument('-e', '--ele', help="ele tag value (default 0.0)", type=float, default=0.0, required=False)
args = parser.parse_args()

# reading input file
print("reading: {}".format(args.input))
tree = ET.parse(args.input)
osm = tree.getroot()

# add missing 'ele' attribute
ele = ET.Element("tag", attrib={"k": "ele", "v": str(args.ele)})
for node in osm.iter("node"):
  has_ele = False
  for tag in node.iter("tag"):
    has_ele = (tag.attrib["k"] == "ele")
    if has_ele:
      break
  if not has_ele:
    print("add missing 'ele' value ({}) to node: {}".format(args.ele, node.attrib["id"]))
    node.append(ele)

# find next available ID
next_id = 0
for child in osm:
  if "id" in child.attrib:
    next_id = max(next_id, int(child.attrib["id"]))
next_id += 1

# update childs with negative ID
for child in osm:
  if "id" in child.attrib:
    id = int(child.attrib["id"])
    if id < 0:
      print("update id {} -> {}".format(id, next_id))
      # update all tags with this ID
      for child in osm.iter():
        if ("id" in child.attrib) and (int(child.attrib["id"]) == id):
          child.attrib["id"] = str(next_id)
        if ("ref" in child.attrib) and (int(child.attrib["ref"]) == id):
          child.attrib["ref"] = str(next_id)
      next_id += 1

# save output file
print("saving: {}".format(args.output))
tree.write(args.output)

print("done")
