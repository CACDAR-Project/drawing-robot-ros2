#!/usr/bin/env python3

import lxml.etree as ET

xml = ET.parse('svg/test.svg')
svg = xml.getroot()
print(svg)

# AttributeError: 'lxml.etree._ElementTree' object has no attribute 'tag'
print(svg.tag)

# TypeError: 'lxml.etree._ElementTree' object is not subscriptable
print(svg[0])

# TypeError: 'lxml.etree._ElementTree' object is not iterable
for child in svg:
    print('width:', svg.get('width'))
    if (child.tag == 'line'):
        print(child.get('x1'))
        print(child.get('x2'))
        print(child.get('y1'))
        print(child.get('y2'))
