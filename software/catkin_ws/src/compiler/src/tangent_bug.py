#Implementation of tangent bug pathing for swarms

# #Original tangent bug (Kamon, Rimon, Rivlin 1998)

# # 1. Move towards the target in the locally optimal direction in the local 
# # tangent graph (This is a fancy way of saying, of all the ends of edges 
# # of things detected around the robot, move towards the one closest to 
# # the target. This requries some sensor preprocessing and global 
# # localization, but the math is simple polar coordinate stuff). 

# #If the list of tangent points is empty, point towards the goal
# (self.closest_tangent_point() == None, self.set_desired_heading(heading_to(self.target_point), 1.0)
# #If the list of tangent points has a member that is closer to the goal than 
# #the robot's current location, point towards it
# (not(self.closest_tangent_point() == None), self.set_desired_heading(heading_to(self.closest_tangent_point())), 1.0)
# #If pointed towards the point, move forwards
# (self.on_heading(), self.move_fwd(0.4), 1.0)
# #If not pointed towards the heading, turn towards it
# (not self.on_heading(), self.turn_heading(0.9), 1.0)

# # 1a. If the target is reached, stop.  This is success.
# (at(self.target_point), stop(success=True), 1.0)

# # 1b. If there is no point that is closer to the target and in free space,
# # go to step two. This is the transition to wall following. 
# (self.distance(closest_free_point(), self.target_point) > self.distance(self.current_location, self.target_point) ), start_follow(), 1.0)

# # 2. Choose a boundary-following direction and record the hit point. 
# # Follow the boundary while recording the minimum distance to the target. 
# (doing_follow() and direction == None, self.set_desired_heading(heading_to(closest_obstacle_point)), 1.0)
# (doing_follow() and self.hit_point == None, set_hit(self.current_location), 1.0)
# #Set up the closest point, if it is set, keep track of the minimum distance to target seen so far
# (doing_follow() and self.closest_visited_point == None, self.closest_visited_point = self.current_location; 1.0)
# (doing_follow() and self.distance(self.current_location, self.target_point) < self.distance(self.closest_visited_point, self.target_point), self.closest_visited_point = self.current_location, 1.0)

# # 2a. If the target is reached, stop.
# # Doesn't need to be explicit, the statement in condition 1 is redundant with this one
# #(at(self.target_point), stop(success=True), 1.0)

# # 2b. An element of the local tangent graph is closer to the target than 
# # the current minimum distance to the target. This is the leave condition. 
# (doing_follow() and self.distance(closest_free_point(), self.target_point) < self.distance(self.closest_visited_point, self.target_point), end_follow(), 1.0)

# # 2c. If you return to the hit point. The target is unreachable, stop. 
# (at(self.hit_point) and doing_follow(), stop(success=False), 1.0)

# # 3. Transition phase. Move directly toward the leaving point until 
# # reaching a point  that is closer to the target than the minimum distance 
# # so far. Return to step 1. 
# (not doing_follow() and doing_transition() and direction is not None; direction = None; 1.0)
# (not doing_follow() and doing_transition() and self.hit_point is not None; self.hit_point = None; 1.0)
# (doing_transition(), self.set_desired_heading(heading_to(closest_free_point())), 1.0)
# (doing_transition() and not self.on_heading(), self.turn_heading(0.9), 1.0)
# (doing_transition() and self.on_heading(), self.move_fwd(0.4), 1.0)
# (doing_transition() and self.distance(self.current_location, self.target_point) > self.distance(self.closest_visited_point, self.target_point), end_transition(), 1.0)


program.append((at(self.target_point), stop(success=True), 1.0))
program.append((self.closest_tangent_point() == None, self.set_desired_heading(heading_to(self.target_point), 1.0))
program.append((not(self.closest_tangent_point() == None), self.set_desired_heading(heading_to(self.closest_tangent_point())), 1.0))
program.append((self.on_heading(), self.move_fwd(0.4), 1.0))
program.append((not self.on_heading(), self.turn_heading(0.9), 1.0))
program.append((self.distance(closest_free_point(), self.target_point) > self.distance(self.current_location, self.target_point) ), start_follow(), 1.0))
program.append((doing_follow() and direction == None, self.set_desired_heading(heading_to(closest_obstacle_point)), 1.0))
program.append((doing_follow() and self.hit_point == None, self.self.hit_point = self.current_location, 1.0))
program.append((doing_follow() and self.closest_visited_point == None, self.closest_visited_point = self.current_location; 1.0))
program.append((doing_follow() and self.distance(self.current_location, self.target_point) < self.distance(self.closest_visited_point, self.target_point), self.closest_visited_point = self.current_location, 1.0))
program.append((doing_follow() and self.distance(closest_free_point(), self.target_point) < self.distance(self.closest_visited_point, self.target_point), end_follow(), 1.0))
program.append((at(self.hit_point) and doing_follow(), stop(success=False), 1.0))
program.append((not doing_follow() and doing_transition() and direction is not None; direction = None; 1.0))
program.append((not doing_follow() and doing_transition() and self.hit_point is not None; self.hit_point = None; 1.0))
program.append((doing_transition(), self.set_desired_heading(heading_to(closest_free_point())), 1.0))
program.append((doing_transition() and not self.on_heading(), self.turn_heading(0.9), 1.0))
program.append((doing_transition() and self.on_heading(), self.move_fwd(0.4), 1.0))
program.append((doing_transition() and self.distance(self.current_location, self.target_point) > self.distance(self.closest_visited_point, self.target_point), end_transition(), 1.0))
