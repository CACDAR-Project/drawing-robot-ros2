#!/usr/bin/env python3

import splipy.curve_factory as cf
import numpy as np
import math

class SVGPathParser():

    def __init__(self, logger, map_point):
        self.logger = logger
        self.map_point = map_point

    def tokenize(self, pathstr):
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
            if c == '+' or c == '-' or c.isdecimal():
                s = c
                isdelim = lambda x: x.isspace() or (x.isalpha() and c != 'e') or x in [',', '+']
                while i < len(pathstr) and not isdelim(c):
                    c = pathstr[i]
                    if not isdelim(c):
                        s = s + c
                    if c.isalpha() and c != 'e':
                        break
                    i += 1
                path.append(s)

        #print(path)
        #input()
        return path

    def parse(self, path):
        '''
        path documentation: https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/d

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
            nonlocal x
            nonlocal y
            p = self.map_point(x,y)
            output.append((p[0],p[1],1.0))
        def setpointdown():
            nonlocal output
            nonlocal x
            nonlocal y
            p = self.map_point(x,y)
            output.append((p[0],p[1],0.0))
        def dropzeros(points):
            out = []
            for x,y in points:
                if x <= 0.0 or y <= 0.0:
                    continue
                out.append([x,y])
            return out
        def appendpoints(points):
            nonlocal output
            for x,y in points:
                p = self.map_point(x,y)
                output.append((p[0],p[1],0.0))
        def lineto(xn,yn):
            nonlocal output
            nonlocal x
            nonlocal y
            setpointdown()
            x = xn
            y = yn
            setpointdown()
        def bezier(control_points):
            nonlocal x
            nonlocal y
            control_points = np.array(control_points)
            #maxval = np.amax(np.absolute(control_points))
            #print('inpput', control_points)
            maxval = np.amax(np.absolute(control_points))
            #print('maxxv', maxval)
            control_points = control_points / maxval #normalize values
            n = 10
            curve = cf.cubic_curve(control_points)
            lin = np.linspace(curve.start(0), curve.end(0), n)
            coordinates = curve(lin)
            coordinates = np.nan_to_num(coordinates)
            coordinates = coordinates * maxval #denormalize values
            coordinates = dropzeros(coordinates)
            #self.logger.info("Appending curve points: {}".format(coordinates))
            #print(coordinates)
            #input()

            #print('start', curve.start(0))
            #print('end', curve.end(0))
            #print('curve', curve)
            #print('lin', lin)
            #print('curvelin', curve(lin))
            #input()
            if len(coordinates) > 0:
                x = coordinates[-1][0]
                y = coordinates[-1][1]
            return coordinates

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
                while True:
                    xn = getnum()
                    yn = getnum()
                    lineto(xn, yn)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "l"):
                while True:
                    xn = x + getnum()
                    yn = y + getnum()
                    lineto(xn, yn)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "H"):
                while True:
                    xn = getnum()
                    lineto(xn, y)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "h"):
                while True:
                    xn = x + getnum()
                    lineto(xn, y)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "V"):
                while True:
                    yn = getnum()
                    lineto(x, yn)
                    if not nextisnum():
                        break
                i += 1
                continue
            if (w == "v"):
                while True:
                    yn = y + getnum()
                    lineto(x, yn)
                    if not nextisnum():
                        break
                i += 1
                continue
            # Cubic Bézier Curve commands
            if (w == "C"):
                while True:
                    # https://github.com/sintef/Splipy/tree/master/examples
                    control_points = [(x,y),
                                      (getnum(),getnum()),
                                      (getnum(),getnum()),
                                      (getnum(),getnum())]
                    coordinates = bezier(control_points)
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
                    coordinates = bezier(control_points)
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
