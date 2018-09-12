#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Modified from severin lemaignan underworlds client example :
# see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
import sys
import underworlds
import rospy
from underworlds.helpers.geometry import get_bounding_box_for_node
from underworlds.types import *
from std_msgs.msg import String
import math

import logging;

logger = logging.getLogger("underworlds.allocentric_relations_manager")
logging.basicConfig(level=logging.INFO)

EPSILON = 0.005  # 5mm
HYSTERESIS_DELTA = 0.02  # 2cm
DELTA = 0.2  # 20cm

CLOSE_MAX_DIST = 1.0  # 1m
NEAR_MAX_DIST = 2.0  # 2m

class AllocentricMonitor(object):
    def __init__(self, ctx, source_world):
        self.source = ctx.worlds[source_world]
        self.source_world_name = source_world
        self.current_situations_map = {}

        self.previous_object_in = {}
        self.object_in = {}

        self.previous_object_above = {}
        self.object_above = {}

        self.previous_object_ontop = {}
        self.object_ontop = {}

        #self.previous_object_bigger = {}
        #self.object_bigger = {}

        self.previous_object_close = {}
        self.object_close = {}

        self.previous_object_near = {}
        self.object_near = {}

        self.log_pub = {"isClose": rospy.Publisher("predicates_log/close", String, queue_size=5),
                        "isNear": rospy.Publisher("predicates_log/near", String, queue_size=5),
                        "isIn": rospy.Publisher("predicates_log/in", String, queue_size=5),
                        "isAbove": rospy.Publisher("predicates_log/above", String, queue_size=5),
                        "isOnTop": rospy.Publisher("predicates_log/ontop", String, queue_size=5),
                        "isBigger": rospy.Publisher("predicates_log/bigger", String, queue_size=5)}

    def bb_center(self, bb):

        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return x1 + x2 / 2, y1 + y2 / 2, z1 + z2 / 2

    def bb_footprint(self, bb):

        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return (x1, y1), (x2, y2)

    def characteristic_dimension(self, bb):
        """ Returns the length of the bounding box diagonal
        """
        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2))

    def distance(self, bb1, bb2):
        """ Returns the distance between the bounding boxes centers.
        """
        x1, y1, z1 = self.bb_center(bb1)
        x2, y2, z2 = self.bb_center(bb2)

        return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2))

    def overlap(self, rect1, rect2):
        '''Overlapping rectangles overlap both horizontally & vertically
        '''
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return self.range_overlap(l1, r1, l2, r2) and \
               self.range_overlap(b1, t1, b2, t2)

    def range_overlap(self, a_min, a_max, b_min, b_max):
        '''Neither range is completely greater than the other

        http://codereview.stackexchange.com/questions/31352/overlapping-rectangles
        '''
        return (a_min <= b_max) and (b_min <= a_max)

    def weakly_cont(self, rect1, rect2):
        '''Obj1 is weakly contained if the base of the object is surrounded
        by Obj2
        '''
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return (l1 >= l2) and (b1 >= b2) and (r1 <= r2) and (t1 <= t2)

    def isbigger(self, bb1, bb2, prev):
        """

        :param bb1:
        :param bb2:
        :return:
        """
        d1 = self.characteristic_dimension(bb1)
        d2 = self.characteristic_dimension(bb2)
        return d1 > d2 + DELTA

    def isabove(self, bb1, bb2, prev=False):
        """
        For obj 1 to be above obj 2:
             - the bottom of its bounding box must be higher that
               the top of obj 2's bounding box
             - the bounding box footprint of both objects must overlap
        :param bb1:
        :param bb2:
        :param prev:
        :return:
        """
        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        if z1 < z2 - EPSILON:
            return False

        return self.overlap(self.bb_footprint(bb1),
                            self.bb_footprint(bb2))

    def isin(self, bb1, bb2, prev=False):
        """ Returns True if bb1 is in bb2.

        To be 'in' bb1 is weakly contained by bb2 and the bottom of bb1 is lower
        than the top of bb2 and higher than the bottom of bb2.
        """
        bb1_min, _ = bb1
        bb2_min, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max
        x3, y3, z3 = bb2_min

        if z1 > z2 + EPSILON:
            return False

        if z1 < z3 - EPSILON:
            return False

        return self.weakly_cont(self.bb_footprint(bb1), self.bb_footprint(bb2))

    def isontop(self, bb1, bb2, prev=False):
        """ For obj 1 to be on top of obj 2:
             - obj1 must be above obj 2
             - the bottom of obj 1 must be close to the top of obj 2
        """
        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        return z1 < z2 + EPSILON and self.isabove(bb1, bb2, prev)

    def isclose(self, bb1, bb2, prev=False):
        """ Returns True if the first object is close to the second.

        """
        dist = self.distance(bb1, bb2)
        dim1 = self.characteristic_dimension(bb1)
        dim2 = self.characteristic_dimension(bb2)

        return dist < CLOSE_MAX_DIST - (dim2/2 + dim1/2)

    def isnear(self, bb1, bb2, prev=False):
        """ Returns True if the first object is near to the second.

        """
        dist = self.distance(bb1, bb2)
        dim1 = self.characteristic_dimension(bb1)
        dim2 = self.characteristic_dimension(bb2)

        return dist < NEAR_MAX_DIST - (dim2/2 + dim1/2)

    def start_predicate(self, timeline, predicate, subject_name, object_name, isevent=False):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub[predicate].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub[predicate].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[allocentric_monitor] Exception occurred : " + str(e))

    def isobject(self, scene, node):
        isobject = False
        if node != scene.rootnode:
            if node.type == MESH:
                if scene.nodes[node.parent].type != CAMERA:
                    isobject = True
        return isobject

    def compute_relations(self, scene):
        boundingboxes = {}
        for node in scene.nodes:
            if self.isobject(scene, node):
                if node.properties["aabb"]:
                    boundingboxes[node] = get_bounding_box_for_node(scene, node)
        self.allocentric_relations(boundingboxes)

    def allocentric_relations(self, boundingboxes):

        self.object_in = {}
        self.object_above = {}
        self.object_ontop = {}
        #self.object_bigger = {}
        self.object_close = {}
        self.object_near = {}

        for n, bb in boundingboxes.items():
            for n2, bb2 in boundingboxes.items():
                if n == n2:
                    continue

                prev_above = False
                if n in self.previous_object_above:
                    if n2 in self.previous_object_above[n]:
                        prev_above = True

                prev_ontop = False
                if n in self.previous_object_ontop:
                    if n2 in self.previous_object_ontop[n]:
                        prev_ontop = True

                prev_close = False
                if n in self.previous_object_close:
                    if n2 in self.previous_object_close[n]:
                        prev_close = True

                prev_near = False
                if n in self.previous_object_near:
                    if n2 in self.previous_object_near[n]:
                        prev_near = True

                prev_in = False
                if n in self.previous_object_in:
                    if n2 in self.previous_object_in[n]:
                        prev_in = True

                # prev_bigger = False
                # if n in self.previous_object_bigger:
                #     if n2 in self.previous_object_bigger[n]:
                #         prev_bigger = True

                if self.isabove(bb, bb2, prev_above):
                    if n in self.object_above:
                        self.object_above[n].append(n2)
                    else:
                        self.object_above[n] = [n2]
                    if self.isontop(bb, bb2, prev_ontop):
                        if n in self.object_ontop:
                            self.object_ontop[n].append(n2)
                        else:
                            self.object_ontop[n] = [n2]

                if self.isclose(bb, bb2, prev_close):
                    if n in self.object_close:
                        self.object_close[n].append(n2)
                    else:
                        self.object_close[n] = [n2]

                if self.isnear(bb, bb2, prev_near):
                    if n in self.object_near:
                        self.object_near[n].append(n2)
                    else:
                        self.object_near[n] = [n2]

                if self.isin(bb, bb2, prev_in):
                    if n in self.object_in:
                        self.object_in[n].append(n2)
                    else:
                        self.object_in[n] = [n2]

                # if self.isbigger(bb, bb2, prev_bigger):
                #     if n in self.object_bigger:
                #         self.object_bigger[n].append(n2)
                #     else:
                #         self.object_bigger[n] = [n2]

        self.compute_situations()

    def compute_situations(self):

        for n, nodes_above in self.object_above.items():
            if n not in self.previous_object_above:
                for n2 in nodes_above:
                    self.start_predicate(self.source.timeline, "isAbove", n.name, n2.name)
            else:
                for n2 in nodes_above:
                    if n2 not in self.previous_object_above[n]:
                        self.start_predicate(self.source.timeline, "isAbove", n.name, n2.name)

        for n, nodes_above in self.previous_object_above.items():
            if n not in self.object_above:
                for n2 in nodes_above:
                    self.end_predicate(self.source.timeline, "isAbove", n.name, n2.name)
            else:
                for n2 in nodes_above:
                    if n2 not in self.object_above[n]:
                        self.end_predicate(self.source.timeline, "isAbove", n.name, n2.name)

        for n, nodes_ontop in self.object_ontop.items():
            if n not in self.previous_object_ontop:
                for n2 in nodes_ontop:
                    self.start_predicate(self.source.timeline, "isOnTop", n.name, n2.name)
            else:
                for n2 in nodes_ontop:
                    if n2 not in self.previous_object_ontop[n]:
                        self.start_predicate(self.source.timeline, "isOnTop", n.name, n2.name)

        for n, nodes_ontop in self.previous_object_ontop.items():
            if n not in self.object_ontop:
                for n2 in nodes_ontop:
                    self.end_predicate(self.source.timeline, "isOnTop", n.name, n2.name)
            else:
                for n2 in nodes_ontop:
                    if n2 not in self.object_ontop[n]:
                        self.end_predicate(self.source.timeline, "isOnTop", n.name, n2.name)

        for n, nodes_close in self.object_close.items():
            if n not in self.previous_object_close:
                for n2 in nodes_close:
                    self.start_predicate(self.source.timeline, "isClose", n.name, n2.name)
            else:
                for n2 in nodes_close:
                    if n2 not in self.previous_object_close[n]:
                        self.start_predicate(self.source.timeline, "isClose", n.name, n2.name)

        for n, nodes_close in self.previous_object_close.items():
            if n not in self.object_close:
                for n2 in nodes_close:
                    self.end_predicate(self.source.timeline, "isClose", n.name, n2.name)
            else:
                for n2 in nodes_close:
                    if n2 not in self.object_close[n]:
                        self.end_predicate(self.source.timeline, "isClose", n.name, n2.name)

        for n, nodes_near in self.object_near.items():
            if n not in self.previous_object_near:
                for n2 in nodes_near:
                    self.start_predicate(self.source.timeline, "isNear", n.name, n2.name)
            else:
                for n2 in nodes_near:
                    if n2 not in self.previous_object_near[n]:
                        self.start_predicate(self.source.timeline, "isNear", n.name, n2.name)

        for n, nodes_near in self.previous_object_near.items():
            if n not in self.object_near:
                for n2 in nodes_near:
                    self.end_predicate(self.source.timeline, "isNear", n.name, n2.name)
            else:
                for n2 in nodes_near:
                    if n2 not in self.object_near[n]:
                        self.end_predicate(self.source.timeline, "isNear", n.name, n2.name)

        for n, nodes_in in self.object_in.items():
            if n not in self.previous_object_in:
                for n2 in nodes_in:
                    self.start_predicate(self.source.timeline, "isIn", n.name, n2.name)
            else:
                for n2 in nodes_in:
                    if n2 not in self.previous_object_in[n]:
                        self.start_predicate(self.source.timeline, "isIn", n.name, n2.name)

        for n, nodes_in in self.previous_object_in.items():
            if n not in self.object_in:
                for n2 in nodes_in:
                    self.end_predicate(self.source.timeline, "isIn", n.name, n2.name)
            else:
                for n2 in nodes_in:
                    if n2 not in self.object_in[n]:
                        self.end_predicate(self.source.timeline, "isIn", n.name, n2.name)

        # for n, nodes_bigger in self.object_bigger.items():
        #     if n not in self.previous_object_bigger:
        #         for n2 in nodes_bigger:
        #             self.start_predicate(self.source.timeline, "isBigger", n.name, n2.name)
        #     else:
        #         for n2 in nodes_bigger:
        #             if n2 not in self.previous_object_bigger[n]:
        #                 self.start_predicate(self.source.timeline, "isBigger", n.name, n2.name)
        #
        # for n, nodes_bigger in self.previous_object_bigger.items():
        #     if n not in self.object_bigger:
        #         for n2 in nodes_bigger:
        #             self.end_predicate(self.source.timeline, "isBigger", n.name, n2.name)
        #     else:
        #         for n2 in nodes_bigger:
        #             if n2 not in self.object_bigger[n]:
        #                 self.end_predicate(self.source.timeline, "isBigger", n.name, n2.name)

        self.previous_object_above = self.object_above
        self.previous_object_ontop = self.object_ontop
        #self.previous_object_bigger = self.object_bigger
        self.previous_object_close = self.object_close
        self.previous_object_near = self.object_near
        self.previous_object_in = self.object_in

    def runOnce(self):
        """
        """
        self.compute_relations(self.source.scene)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.runOnce()
            rate.sleep()

if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)
    import argparse

    parser = argparse.ArgumentParser(description="Monitor allocentric spatial relations")
    parser.add_argument("source_world", help="Underworlds world to monitor")
    args = parser.parse_args()

    rospy.init_node("allocentric_relations_manager", anonymous=True)
    with underworlds.Context("Allocentric Relations Manager") as ctx:
        AllocentricMonitor(ctx, args.source_world).run()

