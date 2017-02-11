#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

# Global Variables

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is new_distting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    dist_sum = 0
    for box, index in state.boxes.items():       # Find nearest storage point to box that is not in restrictions list
      min_distace = 2**31
      for storage in state.storage:
        if (state.restrictions == None) or (state.restrictions[index] == None) or (storage in state.restrictions[index]):
          new_dist = (abs(box[0] - storage[0]) + abs(box[1] - storage[1])) # Calculate manhattan distance between box and storage point
          if new_dist < min_distace:
            min_distace = new_dist
      dist_sum += min_distace

    return dist_sum

def at_storage(state, box, index):
  ''' Checks if box is in an allowed storage or not'''
  
  if state.restrictions:
    return box in state.restrictions[index]
  else:
    return box in state.storage
def corner_deadlock(state, box):
  ''' Checks if box is corner_deadlock, BUT NOT AT GOAL STATE!!, using both the dimensions of map and the obstacles '''

  up_block = (box[1] == 0) or ((box[0], box[1] + 1) in state.obstacles) 
  down_block = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles) 

  left_block = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)
  right_block = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

  return (up_block or down_block) and (left_block or right_block) 

def edge_deadlock(state, box, storage):
  ''' Checks if there is a deadlock due to map walls on a box and a possible storage point '''

  # Check if box is either at the leftmost or rightmost wall, and check if storage is not along that wall
  if ((box[0] == 0) or (box[0] == state.width - 1)) and (box[0] - storage[0] != 0):
      return True  
  # Check if box is either at the topmost or bottommost wall, and check if storage is not along that wall
  elif ((box[1] == state.height - 1) or (box[1] == 0)) and (box[1] - storage[1] != 0):
      return True
  return False

def heur_alternate(state):
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
      
    global prev_boxes
    global prev_heuristic
    dist_sum = 0
    floating_boxes = []

    # Return previous heuristic if the boxes did not change location (saves time)
    try:                                            # Use the try statement for when it is the first time the heuristic is being run (variable might not yet be declared)
      if (prev_boxes == state.boxes):               
        return prev_heuristic
      else:
        prev_boxes = state.boxes
    except NameError:
      prev_boxes = state.boxes

    # Create an array of only boxes that are not at storage
    for box, index in state.boxes.items():
      if not at_storage(state, box, index):
        floating_boxes.append(box)
    if not floating_boxes:
      return 0

    # Begin by iterating through the boxes that are not already at their goal
    for box in floating_boxes:
      if corner_deadlock(state, box):
        prev_heuristic = 2**31
        return 2**31

      elif (state.restrictions == None):
        min_distance = 2**31
        for storage in state.storage:
          if edge_deadlock(state, box, storage):       # If box cant reach one storage point
            prev_heuristic = 2**31
            return 2**31
          # Calculate manhattan distance
          new_dist = (abs(box[0] - storage[0]) + abs(box[1] - storage[1])) # Calculate manhattan distance between box and storage point
          if new_dist < min_distance:
            min_distance = new_dist

      else:                                       # If restrictions exist
        min_distance = 2**31
        i = state.boxes[box]                               
        for storage in state.restrictions[i]: 
          if edge_deadlock(state, box, storage):       # If box cant reach one storage point
            prev_heuristic = 2**31
            return 2**31
          # Calculate manhattan distance
          new_dist = (abs(box[0] - storage[0]) + abs(box[1] - storage[1])) # Calculate manhattan distance between box and storage point
          if new_dist < min_distance:
            min_distance = new_dist
      
      # Take into consideration nearby objects, because clutter can be an indicator of distance to completion
      objects_around = ((box[0]+1, box[1]), (box[0]+1, box[1]+1), (box[0]-1, box[1]+1), (box[0]+1, box[1]-1),
                         (box[0]-1, box[1]-1),(box[0]-1, box[1]), (box[0], box[1]-1), (box[0], box[1]+1))
      dist_sum += min_distance + len(set(state.obstacles)&set(objects_around))
    prev_heuristic = dist_sum
    return prev_heuristic
    

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight*sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 

    # Initialize Time
    time = os.times()[0]
    end_time = time + timebound
    new_timebound = timebound

    # Initialize Searcher
    searcher = SearchEngine(strategy='best_first', cc_level='default')
    searcher.init_search(initial_state, sokoban_goal_state, heur_fn)

    # Perform  search and gather time
    cost_bound = (float("inf"), float("inf"), float("inf"))
    best_result = False
    result = searcher.search(new_timebound)

    # Perform the search while the timebound has not been reached
    while time < end_time:
      if result == False:         # If no result found
        return best_result
      diff_time = os.times()[0] - time
      time = os.times()[0]        # Get current time
      new_timebound = new_timebound - diff_time
      if (result.gval <= cost_bound[0]):
          cost_bound = (result.gval, result.gval, result.gval)          
          best_result = result
      result = searcher.search(new_timebound, cost_bound)
    return best_result

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 

    # Initialize time
    time = os.times()[0]
    end_time = time + timebound
    new_timebound = timebound

    # Initialize search engine
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    searcher = SearchEngine(strategy='custom', cc_level= 'default')
    searcher.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

    # Initialize cost bounds and variables
    cost_bound = (float("inf"), float("inf"), float("inf"))
    best_result = False
    result = searcher.search(timebound)

    # Perform search while within the timebound
    while time < end_time:
      if result == False:     # If no result found
        return best_result
      diff_time = os.times()[0] - time
      time = os.times()[0]    # Get current time
      new_timebound = new_timebound - diff_time
      if (result.gval <= cost_bound[0]):
          cost_bound = (result.gval, result.gval, result.gval)          
          best_result = result
      result = searcher.search(new_timebound, cost_bound)
    
    return best_result


if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 