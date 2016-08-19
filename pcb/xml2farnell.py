from __future__ import print_function

import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

if len(sys.argv) != 3:
    print("Usage: {} <input file> <output file>".format(sys.argv[0]))
    sys.exit()

tree = ET.parse(sys.argv[1])
root = tree.getroot()
part_qtys = defaultdict(int)
part_refs = defaultdict(str)
part_desc = {}

nofarnell = []

for comp in root.iter('comp'):
    ref = comp.get('ref')
    val = comp.findtext('value')
    footprint = comp.findtext('footprint')
    if footprint:
        footprint = footprint.split(":")[-1]
    fields = comp.find('fields')
    found_farnell = False
    if not fields:
        print("Part does not have Farnell ID: ", ref, val)
        if ref[:2] not in ("TP", "GS"):
            nofarnell.append(ref)
        continue
    for field in fields.findall('field'):
        if field.get('name') == "Farnell":
            farnell = field.text
            part_qtys[farnell] += 1
            part_refs[farnell] += " {}".format(ref)
            if farnell not in part_desc:
                part_desc[farnell] = (val, footprint)
            if part_desc[farnell] != (val, footprint):
                part_refs[farnell] = "ERROR Different parts specify same "\
                                     "order number."
            found_farnell = True
    if not found_farnell:
        print("Part does not have Farnell ID: ", ref, val)
        if ref[:2] not in ("TP", "GS"):
            nofarnell.append(ref)

with open(sys.argv[2], 'w', encoding='utf8') as f:
    f.write("Farnell quickpaste format:\n\n")
    for part in part_qtys:
        line_note = "{}x {} {}".format(
            part_qtys[part], part_desc[part][0],
            part_desc[part][1]
        )[:30]
        f.write("{},{},{}\n".format(
            part,
            part_qtys[part],
            line_note
        ))
    f.write("\n\n\nWith component refs:\n\n")
    for part in part_qtys:
        f.write("{}, {}, {} {} {}\n".format(part, part_qtys[part],
                                            part_desc[part][0],
                                            part_desc[part][1],
                                            part_refs[part]))
    if nofarnell:
        f.write("\n\n\nNo Farnell codes:\n")
        f.write(', '.join(nofarnell))
        f.write("\n")
