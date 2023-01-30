#!/usr/bin/env python3

import lxml.etree as ET

class SVGProcessor():
    """
    Class which reads an svg file and creates sequences of points

    ...

    Attributes
    ----------
    primitive_fns : dict
        contains a mapping of SVG primitive names to functions that produce points

    Methods
    -------

    Services
    -------

    Topics
    -------
    """

    def __init__(self, logger):
        self.logger = logger
        # A dict containing svg primitive names mapping to functions that handle them
        self.primitive_fns = {
            # Reference:
            # https://developer.mozilla.org/en-US/docs/Web/SVG/Element
            "line": self.primitive_line,
            "polyline": self.primitive_polyline,
            "polygon": self.primitive_polygon,
        }

    def get_primitive_fn(self, primitive):
        '''
        Looks up the primitive tag name in the dictionary of functions.

            Parameters:
                    primitive (lxml child): the primitive from the svg file
            Returns:
                    primitive_fn ():
        '''
        def log_error(p, _):
           self.logger.error("'{}' not supported".format(p.tag))
           return []
        return self.primitive_fns.get(primitive.tag, log_error)


    def down_and_up(self, points):
        down = (points[0][0], points[0][1], 1)
        up = (points[-1][0], points[-1][1], 1)

        return [down] + points + [up]

    def primitive_line(self, child, map_point):
        p1 = map_point(float(child.get('x1')), float(child.get('y1')))
        p2 = map_point(float(child.get('x2')), float(child.get('y2')))
        return [
            (p1[0],p1[1],0),
            (p2[0],p2[1],0),
            ]

    def primitive_polyline(self, child, map_point):
        points = child.get('points').split(' ')
        points = [(map_point(float(p[0])),
                   map_point(float(p[1])))
                  for p in points.split(',')]
        output = []
        for p in points:
            output.append((p[0],p[1],0))
        return output

    def primitive_polygon(self, child, map_point):
        output = self.primitive_polyline(child, map_point)
        output.append((output[0][0],output[0][1],0))
        return output

    # https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/d
    def path_parser(self, child, map_point):
        path = child.get('d')
        self.logger.info("Parsing path :'{}...' with {} characters".format(path[:40], len(path)))
        self.logger.error("Path parser not implemented")
        return []

    # https://stackoverflow.com/questions/30232031/how-can-i-strip-namespaces-out-of-an-lxml-tree
    def strip_ns_prefix(self, tree):
        #xpath query for selecting all element nodes in namespace
        query = "descendant-or-self::*[namespace-uri()!='']"
        #for each element returned by the above xpath query...
        for element in tree.xpath(query):
            #replace element name with its local name
            element.tag = ET.QName(element).localname
        return tree

    def process_svg(self, svg_path):
        with open(svg_path) as svg_file:
            xml = ET.parse(svg_file)
            svg = xml.getroot()
            svg = self.strip_ns_prefix(svg)

            map_point = self.map_point_function(1000,
                                                1000)
            if 'width' in svg.attrib:
                map_point = self.map_point_function(float(svg.get('width')),
                                                    float(svg.get('height')))
            elif 'viewBox' in svg.attrib:
                # TODO parse viewBox
                pass
            else:
               self.logger.error("Unable to get SVG dimensions")

            motions = []
            for child in svg:
               self.logger.info("Attempting to process SVG primitive:'{}'".format(child.tag))
               primitive_fn = self.primitive_line
               # path can consist of multiple primitives
               if (child.tag == 'path'):
                   for m in self.path_parser(child, map_point):
                       motions.append(m)
               else:
                   primitive_fn = self.get_primitive_fn(child)
                   motions.append(primitive_fn(child, map_point))

            motions_refined = []
            for m in motions:
                if m == []:
                    continue
                self.logger.info("Refining:'{}'".format(m))
                motions_refined.append(self.down_and_up(m))
            return motions_refined

    def translate(self, val, lmin, lmax, rmin, rmax):
        lspan = lmax - lmin
        rspan = rmax - rmin
        val = float(val - lmin) / float(lspan)
        return rmin + (val * rspan)

    def map_point_function(self, x_pixels, y_pixels):
        def map_point(xpix,ypix):
            x = self.translate(xpix, 0, x_pixels, 0, 1)
            y = self.translate(ypix, 0, y_pixels, 0, 1)
            return (x,y)
        return map_point

