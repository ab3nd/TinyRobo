#!/usr/bin/python
import random
import math
import time
import copy
from pprint import pprint

class cSpaceMember:
    def __init__(self, id, oracle):
        #Don't have a world location when starting up
        self.x = None
        self.y = None
        #Don't have a heading when starting up
        self.theta = None
        self.id = id
        #self.neighbors = None
        self.state = {}
        self.isBeacon = False
        self.actions = {}
        #dictionary of dictionaries, first dictionary indexed by 
        #beacon id, next by heard-from, entry is distance
        self.beacons = []
        self.unchangedCount = 0
        self.oracle = oracle
        
    def neighborDistance(self, targetID):
        if targetID in self.oracle.getNeighbors(self.id):
            return self.oracle.getDistance(self.id, targetID)
        else:
            raise ValueError
    
    def becomeBeacon(self):
        self.isBeacon = True
        #Add self entry to beacons
        self.beacons.append([self.id, [self.id], 0])
        
    def broadcastState(self):
        for neighbor in self.oracle.getNeighbors(self.id):
            #Send a beacon message for each of self.beacons
            for beacon in self.beacons:
                #The other end gets their own copy of the message, not references to mine
                space[neighbor].sendBeaconMessage(copy.deepcopy(beacon), self.id)
        
    
    def sendBeaconMessage(self, beaconMessage, fromID):
        #Each beacon message consists of a beacon id, a route to that beacon and the total distance
        #Add ourselves to the route, and the distance to the beacon we heard about it from. 
        newBeaconMessage = [beaconMessage[0], beaconMessage[1], beaconMessage[2] + self.neighborDistance(fromID)]
        newBeaconMessage[1].append(self.id)
        #Check if it's shorter than any other known routes to that beacon
        newRoutes = []
        noMatch = True
        for route in self.beacons:
            if route[0] == newBeaconMessage[0]:
                #It's a route to the same beacon#pprint(self.beacons)
                noMatch = False
                if route[2] > newBeaconMessage[2]:
                    #The existing route was longer, so keep this one
                    print "{2} replacing {0} with {1}".format(route, newBeaconMessage, self.id)
                    newRoutes.append(newBeaconMessage)
                    #We changed the routes, so the unchanged count drops back to zero
                    self.unchangedCount = 0 
                else:
                    #Shorter route, keep the old one
                    newRoutes.append(route)
            else:
                #Route describes a different route from this message
                newRoutes.append(route)
        if noMatch:
            #The route under consideration is to a previously unheardof beacon, so keep it
            #print "{1} adding new route: {0}".format(newBeaconMessage, self.id)
            newRoutes.append(newBeaconMessage)
            #We changed the routes, so the unchanged count drops back to zero
            self.unchangedCount = 0 
        #Update
        self.beacons = newRoutes
        
    def updateCount(self):
        self.unchangedCount += 1
        #print "{1} updates since last change: {0}".format(self.unchangedCount, self.id) 
        
    def createCoordSys(self):
        #This robot is at the origin of the system
        self.x = 0
        self.y = 0
        #This robot is facing straight forward
        self.theta = 0
        
        #Tell all my neighbors their positions
        for neighbor in self.oracle.getNeighbors(self.id):
            bearing = self.oracle.getBearing(self.id, neighbor)
            distance = self.oracle.getDistance(self.id, neighbor)
            nX = distance*math.cos(bearing)
            nY = distance*math.sin(bearing)
            #print "{3} sees {0} at ({1}, {2})".format(neighbor, nX, nY, self.id)
            #Send the "message" to the neighbor
            space[neighbor].tellCoordinates([nX, nY, bearing, self.id])
        print "Told Neighbors"
    
    def tellCoordinates(self, coordData):
        self.x = coordData[0]
        self.y = coordData[1]
        self.theta = coordData[2] - self.oracle.getBearing(self.id, coordData[3])
        #TODO this is where I would propagate to my neighbors
        
    def update(self):
        #This is where we put rules. Rules are composed of a boolean condition, 
        #a thing to do if the boolean condition is true, and a rate to do it at
        if not self.isBeacon and len(self.beacons) == 0:
            #Not a beacon, and haven't heard of any beacons
            #Add the action of becoming a beacon to the actions with a low probability
            self.actions["becomeBeacon"] = (self.becomeBeacon, 0.01)
        if not self.isBeacon and len(self.beacons) > 0:
            # Not a beacon, but have heard of beacons, so don't try to become a beacon. 
            pass
        if True:
            #Always broadcast state
            self.actions["sendUpdates"] = (self.broadcastState, 1.0)
        
        #If I am a beacon, or have seen beacons, start counting updates since the last change
        if self.isBeacon or len(self.beacons) > 0:
            self.actions["countStable"] = (self.updateCount , 1.0)
        
        #There's probably a heuristic for what this threshold should be
        if self.isBeacon and self.unchangedCount > 100:
            self.actions["createCoords"] = (self.createCoordSys, 1.0)
            
        # Run (or don't) all the actions according to their probability
        for action in self.actions.keys():
            if random.random() < self.actions[action][1]:
                #import pdb; pdb.set_trace()
                #Invoke the action
                self.actions[action][0]()
            else:
                del self.actions[action]

class worldPosition:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
                
class RaBOracle:
       
    def __init__(self):
        self.spaceSize = 1000
        self.pairwiseDist = {}
        self.pairwiseBearing = {}
        self.neighbors = {}
        self.realWorld = {}   
    
    def add(self, newID):
        #Tie a (random) world position to an ID
        wp = worldPosition()
        wp.x = random.random() * self.spaceSize
        wp.y = random.random() * self.spaceSize
        wp.theta = random.random() * 2 * math.pi
        self.realWorld[newID] = wp
        
        #New node, so don't know neighbors yet
        self.neighbors[newID] = []
        
        #Update the neighbors, distances, and bearings
        #This will include self-distance
        for otherID in self.realWorld.keys():
            #Distancese first
            dist = self.distance(wp, self.realWorld[otherID])
            #Distance is the same in both directions
            self.pairwiseDist[(otherID, newID)] = self.pairwiseDist[(newID, otherID)] = dist
            
            #Neighborhood is based on distance threshold
            if dist < 200:
                #Neighborhood goes both ways
                self.neighbors[newID].append(otherID)
                self.neighbors[otherID].append(newID)
            
            #Bearing is a little more complicated, but this is essentially the angle that the
            #line from the origin to the point would be, if the starting point were the origin
            #That is, treat the first robot as the origin, and calculate atan2 for the difference
            #in the position of the second robot from the first one
            dx = wp.x - self.realWorld[otherID].x
            dy = wp.y - self.realWorld[otherID].y
            self.pairwiseBearing[(newID, otherID)] = math.atan2(dy, dx) + wp.theta
            #Of course, the other robot sees the difference the other way around, 
            #so negate the signs and use the other robot's heading 
            dx *= -1
            dy *= -1
            self.pairwiseBearing[(otherID, newID)] = math.atan2(dy, dx) + self.realWorld[otherID].theta
            
    #Euclidian distance between two world positions
    def distance(self, wp1, wp2):
        return math.sqrt(pow((wp1.x-wp2.x), 2) + pow((wp1.y - wp2.y), 2))

    def getDistance(self, id1, id2):
        return self.pairwiseDist[(id1, id2)]
    
    def getBearing(self, id1, id2):
        return self.pairwiseBearing[(id1, id2)]    
    
    def getNeighbors(self, id):
        return self.neighbors[id]
    
    def getPosition(self, id):
        return self.realWorld[id]

class renderer:
    def __init__(self):
        self.count = 0
        
    def render(self, space, oracle):
        with open("frame_{0}.dot".format(self.count), "w") as outFile:
            outFile.write("strict digraph space{\n")
            for id in space.keys():
                outFile.write("member_{0}[\n".format(id))
                #These are the robot's idea of it's position, not the world's idea
                labelTheta = " "
                labelX = " "
                labelY = " "
                #if space[id].isBeacon:
                #    import pdb; pdb.set_trace()
                if space[id].theta is not None:
                    labelTheta = "{0:.2f}".format(space[id].theta)
                if space[id].x is not None:
                    labelX = "{0:.2f}".format(space[id].x)
                if space[id].y is not None:
                    labelY = "{0:.2f}".format(space[id].y)
                    
                outFile.write("   label=\"{0}\n({1},{2}) {3}\"\n".format(id, labelX, labelY, labelTheta))
                outFile.write("   orientation={0}\n".format(oracle.getPosition(id).theta * 180/math.pi))
                if space[id].isBeacon:
                    outFile.write("   shape=house\n")
                    outFile.write("   color=red\n")
                else:
                    outFile.write("   shape=house\n")
                #Set the position 
                pos = oracle.getPosition(id)
                outFile.write("   pos=\"{0},{1}!\"\n".format(pos.x, pos.y))
                outFile.write("]\n")
                
                
                for beacon in space[id].beacons:
                    #Generate the dot links for each route.
                    nodes = [str(x) for x in reversed(beacon[1])]
                    for ii in range(len(nodes)-1):
                        outFile.write("member_{0} -> member_{1};\n".format(nodes[ii], nodes[ii+1]))
            outFile.write("}\n\n")
        self.count += 1
    
if __name__ == "__main__":
    
    #Some basic initialization stuff
    random.seed()
    myRend = renderer()
    
    #Create a bunch of cSpaceMembers and set up the space
    oracle = RaBOracle()
    space = {}
    
    members = 30
    for ii in range(members):
        oracle.add(ii)
        #Member with a unique ID
        member = cSpaceMember(ii, oracle)
        space[ii] = member
            
    while True:
        for id in space.keys():
            #print "-- {0} --".format(member[2].id)
            space[id].update()
        myRend.render(space, oracle)
        
        
            