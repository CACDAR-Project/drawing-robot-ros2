#!/usr/bin/env python3

import lxml.etree as ET
import splipy.curve_factory as cf
import numpy as np
import math

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
        # Landscape A4: 210 x 297 mm
        self.paper_width = 297
        self.paper_height = 210

        self.map_point = self.map_point_function(1000,
                                                1000)

    def get_primitive_fn(self, primitive):
        '''
        Looks up the primitive tag name in the dictionary of functions.

            Parameters:
                    primitive (lxml child): the primitive from the svg file
            Returns:
                    primitive_fn ():
        '''
        def log_error(p):
           self.logger.error("'{}' not supported".format(p.tag))
           return []
        return self.primitive_fns.get(primitive.tag, log_error)


    def down_and_up(self, points):
        down = (points[0][0], points[0][1], 1)
        up = (points[-1][0], points[-1][1], 1)

        return [down] + points + [up]

    def primitive_line(self, child):
        p1 = self.map_point(float(child.get('x1')), float(child.get('y1')))
        p2 = self.map_point(float(child.get('x2')), float(child.get('y2')))
        return [
            (p1[0],p1[1],0),
            (p2[0],p2[1],0),
            ]

    def primitive_polyline(self, child):
        points = child.get('points').split(' ')
        points = [(self.map_point(float(p[0]),float(p[1]))) for p in points.split(',')]
        output = []
        for p in points:
            output.append((p[0],p[1],0))
        return output

    def primitive_polygon(self, child):
        output = self.primitive_polyline(child)
        output.append((output[0][0],output[0][1],0))
        return output

    # https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/d
    def path_parser(self, child):
        '''

        MoveTo: M, m
        LineTo: L, l, H, h, V, v
        Cubic Bézier Curve: C, c, S, s
        Quadratic Bézier Curve: Q, q, T, t
        Elliptical Arc Curve: A, a
        ClosePath: Z, z

            Parameters:
                    primitive (lxml child): the primitive from the svg file
            Returns:
                    primitive_fn ():
        '''
        pathstr = child.get('d')

        # Tokenizer
        self.logger.info("Tokenizing path :'{}...' with {} characters".format(pathstr[:40], len(pathstr)))
        path = []
        i = 0
        while i < len(pathstr):
            c = pathstr[i]
            i += 1
            # Single letter commands
            if c.isalpha():
                path.append(c)

            # Numbers
            if c == '-' or c.isdecimal():
                s = c
                while i < len(pathstr) and not (c.isspace() or c == ','):
                    c = pathstr[i]
                    if c != ',' and not c.isspace():
                        s = s + c
                    i += 1
                path.append(s)

        # Parser
        self.logger.info("Parsing path :'{}...' with {} tokens".format(path[:20], len(path)))
        x = 0.0
        y = 0.0
        i = 0
        output = []
        def getnum():
            nonlocal i
            i += 1
            return float(path[i])

        def isfloat(element):
            #If you expect None to be passed:
            if element is None:
                return False
            try:
                float(element)
                return True
            except ValueError:
                return False

        def nextisnum():
            return i + 1 < len(path) and isfloat(path[i + 1])
        def setpointup():
            nonlocal output
            p = self.map_point(x,y)
            output.append((p[0],p[1],1.0))
        def setpointdown():
            nonlocal output
            p = self.map_point(x,y)
            output.append((p[0],p[1],0.0))
        def appendpoints(points):
            nonlocal output
            for x,y in points:
                p = self.map_point(x,y)
                output.append((p[0],p[1],0.0))


        while i < len(path):
            w = path[i]
            # MoveTo commands
            if (w == "M"):
                setpointup()
                x = getnum()
                y = getnum()
                setpointup()
                i += 1
                continue
            if (w == "m"):
                setpointup()
                x += getnum()
                y += getnum()
                setpointup()
                i += 1
                continue
            # LineTo commands
            if (w == "L"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "l"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "H"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "h"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "V"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "v"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            # Cubic Bézier Curve commands
            if (w == "C"):
                while True:
                    # https://github.com/sintef/Splipy/tree/master/examples
                    control_points = [(x,y),
                                      (getnum(),getnum()),
                                      (getnum(),getnum()),
                                      (getnum(),getnum())]
                    control_points = np.array(control_points)
                    maxval = np.amax(control_points)
                    control_points = control_points / maxval #normalize values
                    n = 50
                    curve = cf.cubic_curve(control_points)
                    lin = np.linspace(curve.start(0), curve.end(0), n)
                    coordinates = curve(lin)
                    coordinates = np.nan_to_num(coordinates)
                    coordinates = coordinates * maxval #denormalize values
                    #self.logger.info("Appending curve points: {}".format(coordinates))
                    x = coordinates[-1][0]
                    y = coordinates[-1][1]
                    appendpoints(coordinates)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "c"):
                while True:
                    # https://github.com/sintef/Splipy/tree/master/examples
                    control_points = [(x,y),
                                      (x + getnum(), y + getnum()),
                                      (x + getnum(), y + getnum()),
                                      (x + getnum(), y + getnum())]
                    control_points = np.array(control_points)
                    maxval = np.amax(control_points)
                    control_points = control_points / maxval #normalize values
                    n = 50
                    curve = cf.cubic_curve(control_points)
                    lin = np.linspace(curve.start(0), curve.end(0), n)
                    coordinates = curve(lin)
                    coordinates = np.nan_to_num(coordinates)
                    coordinates = coordinates * maxval #denormalize values
                    #print("got:", coordinates)
                    #exit()
                    #self.logger.info("Appending curve points: {}".format(coordinates))
                    x = coordinates[-1][0]
                    y = coordinates[-1][1]
                    appendpoints(coordinates)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "S"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "s"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            # Quadratic Bézier Curve commands
            if (w == "Q"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "q"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "T"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "t"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            # Elliptical arc commands
            if (w == "A"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            if (w == "a"):
                self.logger.error("SVG path parser '{}' not implemented".format(w))
            # ClosePath commands
            if (w == "Z" or w == "z"):
               #TODO draw line if start and end point not are the same
               i += 1
               continue

            self.logger.error("SVG path parser panic mode at '{}'".format(w))

            i += 1
        self.logger.info("Finished parsing path :'{}...' with {} points".format(output[:3], len(output)))
        return output

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

            if 'viewBox' in svg.attrib:
                vb = svg.get('viewBox').split(' ')
                self.map_point = self.map_point_function(float(vb[2]),
                                                         float(vb[3]))
                self.logger.info("Got width:{} and height:{} from viewBox".format(vb[2],vb[3]))
            elif 'width' in svg.attrib:
                self.map_point = self.map_point_function(float(svg.get('width')),
                                                         float(svg.get('height')))
                self.logger.info("Got width:{} and height:{} from width and height attributes".format(svg.get('width'),svg.get('height')))
            else:
               self.logger.error("Unable to get SVG dimensions")

            motions = []
            def process_tags(svg):
                nonlocal motions
                for child in svg:
                    self.logger.debug("Attempting to process SVG primitive:'{}'".format(child.tag))
                    primitive_fn = self.primitive_line
                    # path can consist of multiple primitives
                    if (child.tag == 'path'):
                        #for m in self.path_parser(child):
                        #    motions.append(m)
                        motions.append(self.path_parser(child))
                    elif (child.tag == 'g'):
                        self.logger.info("Recursively processing SVG primitive:'{}'".format(child.tag))
                        process_tags(child)
                    else:
                        primitive_fn = self.get_primitive_fn(child)
                        motions.append(primitive_fn(child))

            process_tags(svg)

            motions_refined = []
            for m in motions:
                if m == []:
                    continue

                mm = self.remove_homes(m)
                mm = self.remove_redundant(mm)

                #self.logger.info("Refining:'{}...'".format(m[:3]))
                motions_refined.append(self.down_and_up(mm))

            # Move to home at end
            motions_refined.append([(0.0, 0.0, 1.0)])
            return motions_refined

    def remove_homes(self, motion):
        # Remove unnecessary moves home
        mm = []
        for p in motion:
            if p[0] <= 0.0 and p[1] <= 0.0:
                continue
            mm.append(p)
        return mm

    def remove_redundant(self, motion):
        # Remove points that are too close to the previous point, specified by the tolerance
        mm = []
        tolerance = 0.001
        prev = (-1, -1, 0)
        for i, p in enumerate(motion):
            x = p[0]
            y = p[1]
            z = p[2]
            px = prev[0]
            py = prev[1]
            pz = prev[2]
            xdiff = abs(x - px)
            ydiff = abs(y - py)
            zdiff = abs(z - pz)
            if xdiff < tolerance and ydiff < tolerance and zdiff < tolerance:
                continue
            prev = p
            mm.append(p)
        return mm

    def translate(self, val, lmin, lmax, rmin, rmax):
        lspan = lmax - lmin
        rspan = rmax - rmin
        val = float(val - lmin) / float(lspan)
        return rmin + (val * rspan)

    def map_point_function(self, x_pixels, y_pixels):
        x_offset = 0
        y_offset = 0
        if (x_pixels > y_pixels):
            ratio =  self.paper_height / self.paper_width
            y_offset = x_pixels * ratio - y_pixels
        else:
            ratio = self.paper_width / self.paper_height
            x_offset = y_pixels * ratio - x_pixels

        def map_point(xpix,ypix):
            x = self.translate(xpix, 0, x_pixels + x_offset, 0, 1)
            y = self.translate(ypix, 0, y_pixels + y_offset, 0, 1)
            return (x,y)
        return map_point

