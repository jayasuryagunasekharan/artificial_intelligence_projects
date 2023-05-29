import argparse
from pythonds.basic.stack import Stack
import numpy as np
from datetime import datetime

#parse the arguments passed in the terminal
def parse_args():
    par = argparse.ArgumentParser()
    #start file
    par.add_argument("st_file")
    #goal file
    par.add_argument("gl_file")
    #enter the method name which needs to be checked
    par.add_argument("method_name", nargs="?", default="a*", choices=["bfs", "ucs", "dfs", "dls", "ids", "greedy", "a*"])
    #dump file which print the details of the search
    par.add_argument("dmp_flg", nargs="?", default="false", choices=["true", "false"])
    return par.parse_args()

#moving the blank location UP
def mv_up(state):
    updated_state = state[:]
    i = updated_state.index(0)

    if i not in [0, 1, 2]:
        temp = updated_state[i - 3]
        updated_state[i - 3] = updated_state[i]
        updated_state[i] = temp
        return updated_state
    else:
        return None

#calculation of the cost while moving UP
def mv_up_cost(state, cost):
    updated_state = state[:]
    i = updated_state.index(0)
    current_cost = cost

    if i not in [0, 1, 2]:
        temp = updated_state[i - 3]
        updated_state[i - 3] = updated_state[i]
        updated_state[i] = temp
        current_cost += temp
        return current_cost
    else:
        return None

#moving the blank location DOWN
def mv_down(state):
    updated_state = state[:]
    i = updated_state.index(0)

    if i not in [6, 7, 8]:
        temp = updated_state[i + 3]
        updated_state[i + 3] = updated_state[i]
        updated_state[i] = temp
        return updated_state
    else:
        return None

#calculation of the cost while moving DOWN
def mv_down_cost(state, cost):
    updated_state = state[:]
    i = updated_state.index(0)
    current_cost = cost

    if i not in [6, 7, 8]:
        temp = updated_state[i + 1]
        updated_state[i + 1] = updated_state[i]
        updated_state[i] = temp
        current_cost += temp
        return current_cost
    else:
        return None

#moving the blank location LEFT
def mv_left(state):
    updated_state = state[:]
    i = updated_state.index(0)

    if i not in [0, 3, 6]:
        temp = updated_state[i - 1]
        updated_state[i - 1] = updated_state[i]
        updated_state[i] = temp
        return updated_state
    else:
        return None

#calculation of the cost while moving LEFT
def mv_left_cost(state, cost):
    updated_state = state[:]
    i = updated_state.index(0)
    current_cost = cost

    if i not in [0, 3, 6]:
        temp = updated_state[i - 3]
        updated_state[i - 3] = updated_state[i]
        updated_state[i] = temp
        current_cost += temp
        return current_cost
    else:
        return None

#moving the blank location RIGHT
def mv_right(state):
    updated_state = state[:]
    i = updated_state.index(0)

    if i not in [2, 5, 8]:
        temp = updated_state[i + 1]
        updated_state[i + 1] = updated_state[i]
        updated_state[i] = temp
        return updated_state
    else:
        return None

#calculation of the cost while moving RIGHT
def mv_right_cost(state, cost):
    updated_state = state[:]
    i = updated_state.index(0)
    current_cost = cost

    if i not in [2, 5, 8]:
        temp = updated_state[i + 1]
        updated_state[i + 1] = updated_state[i]
        updated_state[i] = temp
        current_cost += temp
        return current_cost
    else:
        return None

#create a node for storing state of the game
def create_rod(condition, daddy, movement, depth, cost):
    return Node(condition, daddy, movement, depth, cost)


#node expansion for checking all possible states
def expand_rod(node):
    all_expanded_nodes = [create_rod(mv_up(node.condition), node, "up", node.depth + 1, mv_up_cost(node.condition, node.cost)),
                      create_rod(mv_down(node.condition), node, "down", node.depth + 1, mv_down_cost(node.condition, node.cost)),
                      create_rod(mv_left(node.condition), node, "left", node.depth + 1, mv_left_cost(node.condition, node.cost)),
                      create_rod(mv_right(node.condition), node, "right", node.depth + 1, mv_right_cost(node.condition, node.cost))]

    all_expanded_nodes = [node for node in all_expanded_nodes if node.condition is not None]
    return all_expanded_nodes

def bfs(start, goal, dmp_flg):
    goal = goal
    start_state = create_rod(start,None,None,0,0)
    sum_cost = []
    path = []
    fringe_queue = [start_state]
    present_node = fringe_queue.pop(0)

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        fringe_queue.extend(expand_rod(present_node))
        nodes_expanded += 1
        present_node = fringe_queue.pop(0)
        sum_cost.insert(0, present_node.cost)
        nodes_popped += 1

        if len(fringe_queue) > max_fringe_size:
            max_fringe_size = len(fringe_queue)
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0,present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-BFS.txt'
        with open(filename, 'w') as f:
            f.write('Fringe contents:\n')
            for node in fringe_queue:
                f.write(f'{node}\n')
            f.write('\nClosed set contents:\n')
            for state in present_node.condition:
                f.write(f'{state}\n')
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def dfs(start, goal, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    fringe_stack = Stack()
    sum_cost = []
    path = []
    fringe_stack.push(start_node)
    visited = set()
    present_node = fringe_stack.pop()

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        temp = expand_rod(present_node)

        for item in temp:
            if tuple(item.condition) not in visited:
                visited.add(tuple(item.condition))
                fringe_stack.push(item)
                sum_cost.insert(0, item.cost)
        present_node = fringe_stack.pop()

        nodes_popped += 1
        if fringe_stack.size() > max_fringe_size:
            max_fringe_size = fringe_stack.size()
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-DFS.txt'
        with open(filename, 'w') as f:
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def dls(start, goal, depth, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    fringe_stack = Stack()
    sum_cost = []
    path = []
    fringe_stack.push(start_node)
    visited = set()
    present_node = fringe_stack.pop()

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        temp = expand_rod(present_node)

        for item in temp:
            if tuple(item.condition) not in visited:
                visited.add(tuple(item.condition))
                fringe_stack.push(item)
                sum_cost.insert(0, item.cost)
            if item.depth == depth:
                return None

            nodes_popped += 1
            if fringe_stack.size() > max_fringe_size:
                max_fringe_size = fringe_stack.size()
            nodes_generated += len(expand_rod(present_node))
            nodes_expanded += 1

        present_node = fringe_stack.pop()

        if present_node.depth > depth:
            return None


    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-DLS.txt'
        with open(filename, 'w') as f:
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def uniform_cost(start, goal, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    fringe_queue = []
    path = []
    sum_cost = []
    fringe_queue.append(start_node)
    present_node = fringe_queue.pop(0)

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        temp = expand_rod(present_node)
        for item in temp:
            item.depth += present_node.depth
            item.cost += present_node.cost
            fringe_queue.append(item)
            print(np.array(item.condition).reshape(3,3), "\n")
            print("Cost        : ", item.cost)
            print("Depth       : ", item.depth)
            print("Current Path: ", item.movement)
            sum_cost.insert(0, item.cost)
        fringe_queue.sort(key=lambda x: x.cost)
        present_node = fringe_queue.pop(0)

        nodes_popped += 1
        if len(fringe_queue) > max_fringe_size:
            max_fringe_size = len(fringe_queue)
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-UCS.txt'
        with open(filename, 'w') as f:
            f.write('Fringe contents:\n')
            for node in fringe_queue:
                f.write(f'{node}\n')
            f.write('\nClosed set contents:\n')
            for state in present_node.condition:
                f.write(f'{state}\n')
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def ids(start, goal, dmp_flg):
    depth = 0
    max_fringe_size = 0
    while True:
        print(depth)
        result = depth_limited_search(start, goal, depth, max_fringe_size, dmp_flg)
        if result is not None:
            return result
        print("Depth: ", depth)
        depth += 10000

def depth_limited_search(start, goal, depth_limit, max_fringe_size, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    fringe_stack = Stack()
    path = []
    sum_cost = []
    fringe_stack.push(start_node)

    visited_node = set()
    present_node = fringe_stack.pop()

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1

    while present_node.condition != goal:

        if present_node.depth == depth_limit:
            return None
        temp = expand_rod(present_node)

        for item in temp:
            if tuple(item.condition) not in visited_node:
                visited_node.add(tuple(item.condition))
                fringe_stack.push(item)
                sum_cost.insert(0, item.cost)
        if fringe_stack.size() > max_fringe_size:
            max_fringe_size = fringe_stack.size()
        present_node = fringe_stack.pop()

        nodes_popped += 1
        if fringe_stack.size() > max_fringe_size:
            max_fringe_size = fringe_stack.size()
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-IDS.txt'
        with open(filename, 'w') as f:
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def greedy(start, goal, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    visited = set()
    fringe = []
    path = []
    sum_cost = []
    fringe.append(start_node)
    present_node = fringe.pop(0)

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        temp = expand_rod(present_node)
        for item in temp:
            if tuple(item.condition) not in visited:
                visited.add(tuple(item.condition))
                fringe.append(item)
                sum_cost.insert(0, item.cost)
                h(item, goal)
        fringe.sort(key=lambda x: x.depth)
        fringe.sort(key=lambda x: x.heuristic)
        present_node = fringe.pop(0)

        nodes_popped += 1
        if len(fringe) > max_fringe_size:
            max_fringe_size = len(fringe)
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-GREEDY.txt'
        with open(filename, 'w') as f:
            f.write('Fringe contents:\n')
            for node in fringe:
                f.write(f'{node}\n')
            f.write('\nClosed set contents:\n')
            for state in present_node.condition:
                f.write(f'{state}\n')
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def a_star(start, goal, dmp_flg):
    start_node = create_rod(start, None, None, 0, 0)
    fringe = []
    closed_set = set()
    path = []
    sum_cost = []
    fringe.append(start_node)
    present_node = fringe.pop(0)

    nodes_popped = 0
    nodes_expanded = 0
    nodes_generated = 1
    max_fringe_size = 1

    while present_node.condition != goal:
        closed_set.add(tuple(present_node.condition))
        fringe.extend(expand_rod(present_node))
        for item in fringe:
            h(item, goal)
            sum_cost.insert(0, item.cost)
            item.heuristic += item.depth
        fringe.sort(key=lambda x: x.heuristic)
        while len(fringe) > 0 and tuple(fringe[0].condition) in closed_set:
            fringe.pop(0)
        if len(fringe) == 0:
            return None
        present_node = fringe.pop(0)
        nodes_popped += 1

        if len(fringe) > max_fringe_size:
            max_fringe_size = len(fringe)
        nodes_generated += len(expand_rod(present_node))
        nodes_expanded += 1

    while present_node.daddy is not None:
        path.insert(0, present_node.movement)
        present_node = present_node.daddy

    print_solution_path(start, goal, path, sum_cost[0])

    if dmp_flg:
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'trace-{timestamp}-A_STAR.txt'
        with open(filename, 'w') as f:
            f.write('Fringe contents:\n')
            for node in fringe:
                f.write(f'{node}\n')
            f.write('\nClosed set contents:\n')
            for state in present_node.condition:
                f.write(f'{state}\n')
            f.write(f'\nNodes generated: {nodes_generated}\n')
            f.write(f'Nodes expanded: {nodes_expanded}\n')
            f.write(f'Nodes popped: {nodes_popped}\n')
            f.write(f'Maximum fringe size: {max_fringe_size}\n\n')

    return path

def h(node, goal):
    distance_match=0
    for i in range(0,9):
        if node.condition[i] != goal[i]:
            distance_match+=1
    node.heuristic=distance_match

class Node:
    def __init__(self, condition, daddy, movement, depth, cost):
        self.condition = condition
        self.daddy = daddy
        self.movement = movement
        self.depth = depth
        self.cost = cost
        self.heuristic = None

def print_solution_path(start_state, goal_state, solution_path, cost):
    current_state = start_state[:]
    for i, move in enumerate(solution_path):
        print(f"Level {i} State: \n {np.array(current_state).reshape(3, 3)} -----> Move 0 to {move} in next iteration")
        print("---" * 20)
        if move == 'up':
            current_state = mv_up(current_state)
        elif move == 'down':
            current_state = mv_down(current_state)
        elif move == 'left':
            current_state = mv_left(current_state)
        elif move == 'right':
            current_state = mv_right(current_state)
        else:
            print(f"Invalid move: {move}", "\n")
        if current_state == goal_state:
            print(f"Level {i + 1} State: \n {np.array(current_state).reshape(3, 3)}")
            print("-X-" * 20)
            print("Cost: ", cost)
            print("Goal state reached!\n")
            break
    else:
        print("Solution path does not lead to the goal state.", "\n")

def readfile_start(filename):
    with open(filename) as f:
        data = f.read().strip()
        state = [int(x) for x in data.split()]
    return state

def readfile_goal(filename):
    with open(filename) as f:
        data = f.read().strip()
        state = [int(x) for x in data.split()]
    return state

def main():
    args = parse_args()

    dmp_flg = args.dmp_flg

    start_state = readfile_start(args.st_file)
    goal_state = readfile_goal(args.gl_file)


    if args.method_name == 'bfs':
        solution = bfs(start_state, goal_state, dmp_flg)
    elif args.method_name == 'dfs':
        solution = dfs(start_state, goal_state, dmp_flg)
    elif args.method_name == 'dls':
        print("Enter the depth limit: ")
        num = int(input())
        solution = dls(start_state, goal_state, num, dmp_flg)
    elif args.method_name == 'ucs':
        solution = uniform_cost(start_state, goal_state, dmp_flg)
    elif args.method_name == 'ids':
        solution = ids(start_state, goal_state, dmp_flg)
    elif args.method_name == 'greedy':
        solution = greedy(start_state, goal_state, dmp_flg)
    elif args.method_name == 'a*':
        solution = a_star(start_state, goal_state, dmp_flg)
    else:
        print(f"Invalid method_name: {args.method_name}", "\n")

    if solution is None:
        print("No solution found", "\n")
    elif solution == [None]:
        print("Start node was the goal!", "\n")
    else:
        print("Path: ", solution)
        print("Solution found at Depth: ", len(solution), "\n")

if __name__ == "__main__":
    main()