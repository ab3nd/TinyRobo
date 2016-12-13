#!/usr/bin/python
import random
import math
import time
import copy
from pprint import pprint

class cSpaceMember:
    def __init__(self, id):
        self.x = None
        self.y = None
        self.id = id
        self.neighbors = None
        self.state = {}
        self.isBeacon = False
        self.actions = {}
        #dictionary of dictionaries, first dictionary indexed by 
        #beacon id, next by heard-from, entry is distance
        self.beacons = []
        
    def setNeighbors(self, neighbors):
        self.neighbors = neighbors
    
    def neighborDistance(self, targetID):
        for neighbor, dist in self.neighbors:
            #print "{0} --> {1}, {2}".format(self.id, neighbor.id, dist)
            if neighbor.id == targetID:
                return dist
        raise ValueError
    
    def becomeBeacon(self):
        self.isBeacon = True
        #Add self entry to beacons
        self.beacons.append([self.id, [self.id], 0])
        
    def broadcastState(self):
        for neighbor in self.neighbors:
            #Send a beacon message for each of self.beacons
            for beacon in self.beacons:
                #The other end gets their own copy of the message, not references to mine
                neighbor[0].sendBeaconMessage(copy.deepcopy(beacon), self.id)
        
    
    def sendBeaconMessage(self, beaconMessage, fromID):
        #Each beacon message consists of a beacon id, a route to that beacon and the total distance
        #Add ourselves to the route, and the distance to the beacon we heard about it from. 
        newBeaconMessage = [beaconMessage[0], beaconMessage[1], beaconMessage[2] + self.neighborDistance(fromID)]
        newBeaconMessage[1].append(self.id)
        #Check if it's shorter than any other known routes to that beacon
        newRoutes = []
        noMatch = True
        change = False
        for route in self.beacons:
            if route[0] == newBeaconMessage[0]:
                #It's a route to the same beacon
                noMatch = False
                if route[2] > newBeaconMessage[2]:
                    #The existing route was longer, so keep this one
                    print "{2} replacing {0} with {1}".format(route, newBeaconMessage, self.id)
                    newRoutes.append(newBeaconMessage)                    
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
        #Update
        self.beacons = newRoutes
        #pprint(self.beacons)
        
        
    def update(self):
        #This is where we put rules. Rules are composed of a boolean condition, 
        #a thing to do if the boolean condition is true, and a rate to do it at
        if not self.isBeacon and len(self.beacons) == 0:
            #Not a beacon, and haven't heard of any beacons
            #Add the action of becoming a beacon to the actions with a very low probability
            self.actions["becomeBeacon"] = (self.becomeBeacon, 0.001)
        if not self.isBeacon and len(self.beacons) > 0:
            # Not a beacon, but have heard of beacons, so don't try to become a beacon. 
            pass
        if True:
            #Always broadcast state
            self.actions["sendUpdates"] = (self.broadcastState, 1.0)
            
        # Run (or don't) all the actions according to their probability
        for action in self.actions.keys():
            if random.random() < self.actions[action][1]:
                #import pdb; pdb.set_trace()
                #Invoke the action
                self.actions[action][0]()
            else:
                del self.actions[action]
        
        
        
#Euclidian distance
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x1-x2), 2) + pow((y1 - y2), 2))

class renderer:
    def __init__(self):
        self.count = 0
        
    def render(self, space):
        with open("frame_{0}.dot".format(self.count), "w") as outFile:
            outFile.write("strict digraph space{\n")
            for xPos, yPos, member in space:
                outFile.write("member_{0}[\n".format(member.id))
                outFile.write("   label={0}\n".format(member.id))
                if member.isBeacon:
                    outFile.write("   shape=doublecircle\n")
                    outFile.write("   color=red\n")
                else:
                    outFile.write("   shape=circle\n")
                outFile.write("   pos=\"{0},{1}!\"\n".format(xPos, yPos))
                outFile.write("]\n")
                
                
                for beacon in member.beacons:
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
    
    #Create a bunch of cSpaceMembers in the space
    space = []
    members = 30
    spaceSize = 1000
    for ii in range(members):
        space.append((spaceSize * random.random(), spaceSize * random.random(), cSpaceMember(ii)))
        
    #Set up the neighbors based on distance
    for fromMember in space:
        neighbors = []
        for toMember in space:
            d = distance(fromMember[0], fromMember[1], toMember[0], toMember[1])
            if d < 300:
                neighbors.append((toMember[2], d))
        fromMember[2].setNeighbors(neighbors)
        print "{0} has {1} neighbors".format(fromMember[2].id, len(neighbors)) 
    
    while True:
        for member in space:
            #print "-- {0} --".format(member[2].id)
            member[2].update()
        myRend.render(space)
        
        
            