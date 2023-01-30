#!/usr/bin/env python3

import rclpy
import lxml.etree as ET

class SVGProcessor():

    def __init__(self, logger):
        self.logger = logger
        # A dict containing svg primitive names mapping to functions that handle them
        self.primitives = {
            # Reference:
            # https://developer.mozilla.org/en-US/docs/Web/SVG/Element
            #"line": lambda p: print("LINE"),
        }

    def get_primitive(self, primitive):
       log_error = lambda p: self.logger.error("'{}' not supported".format(p.tag))
       return self.primitives.get(primitive.tag, log_error)

    def process_svg(self, svg_path):
        with open(svg_path) as svg:
            xml = ET.parse(svg)
            svg = xml.getroot()

            for child in svg:
               f = self.get_primitive(child)
               f(child)

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
